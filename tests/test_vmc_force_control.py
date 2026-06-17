#!/usr/bin/env python3
"""
单腿 VMC 力控测试。

测试虚拟模型控制（VMC）中足端力 → 关节力矩的映射和实际控制效果。

测试流程：
  0  通信检查    — 读取电机状态
  1  起立        — S 曲线从当前姿态过渡到站立姿态
  2  等待确认    — 按 Enter 继续
  3  力控测试    — 给定目标足端力，计算并施加关节力矩
  4  卸力        — 锁定电机

用法：
  python3 tests/test_vmc_force_control.py                # RF 实机
  python3 tests/test_vmc_force_control.py --leg 1        # 指定腿
  python3 tests/test_vmc_force_control.py --sim          # 仿真
  python3 tests/test_vmc_force_control.py --force_z -20  # 指定竖直力
"""

import argparse
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Controller', 'build'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Module', 'motors', 'build'))

try:
    import vmc_controller_py as vmc
except ImportError:
    print("请先编译 Controller")
    sys.exit(1)

HAS_MOTORS = False
try:
    import motors_py
    HAS_MOTORS = True
except (ImportError, RuntimeError):
    pass

# ════════════════════════════════════
# 配置
# ════════════════════════════════════
LEG_NAMES = {0: "LF", 1: "RF", 2: "LB", 3: "RB"}
JOINT_NAMES = {0: "ABD", 1: "HIP", 2: "KNEE"}
MOTOR_MAP = {0: (1, 2, 3), 1: (None, 1, 2), 2: (7, 8, 9), 3: (10, 11, 12)}

# ── 零点偏移 (rad): q_kinematic = JOINT_SIGN * (q_motor_raw - ZERO_OFFSET) ──
ZERO_OFFSET = [0.0, 0.0, -0.8880]   # ABD, HIP, KNEE: knee ref=-50.88deg

# ── 电机方向修正: +1 同向, -1 反向 ──
JOINT_SIGN = [1.0, -1.0, -1.0]      # ABD, HIP, KNEE

# ── 安全参数 ──
SAFE = {
    "kp_stand": 100.0,
    "kd_stand": 5.0,
    "kp_trans": 80.0,
    "kd_trans": 3.0,
    "kp_force": 0.0,      # 力控时位置刚度为0
    "kd_force": 2.0,      # 小阻尼保持稳定
    "max_pos_err": 0.4,
    "max_torque": 15.0,   # 最大力矩限幅 (N·m)
    "stand_up_time": 2.0,
    "dt": 0.005,          # 200Hz
}


# ════════════════════════════════════
# 硬件接口
# ════════════════════════════════════

class SingleLegInterface:
    def __init__(self, leg, sim, dx=0.06, dy=0.082):
        self.leg = leg
        self.sim = sim
        self.motors = {}
        self.q_sim = [0.0, 0.0, 0.0]
        self.tau_sim = [0.0, 0.0, 0.0]

        x_sign = 1.0 if leg in (0, 1) else -1.0
        y_sign = 1.0 if leg in (0, 2) else -1.0
        self.dx_s = x_sign * dx
        self.dy_s = y_sign * dy
        self.kin = vmc.LegKinematics(0.2125, 0.25025, self.dx_s, self.dy_s)
        self.L1, self.L2 = 0.2125, 0.25025

        if not sim and HAS_MOTORS:
            self._init_real()

    def _init_real(self):
        ids = MOTOR_MAP.get(self.leg, (1, 2, 3))
        for j, mid in enumerate(ids):
            if mid is None:
                continue
            m = motors_py.MotorDriver.create_motor(mid, "CAN", "can0", "LRO_CAN", 2)
            m.init_motor()
            self.motors[j] = m

    def read(self):
        """返回运动学坐标系下的关节角 [abd, hip, knee]。"""
        if self.sim:
            return list(self.q_sim)
        q = [0.0, 0.0, 0.0]
        for j, m in self.motors.items():
            m.refresh_motor_status()
            raw = m.get_motor_pos()
            q[j] = JOINT_SIGN[j] * (raw - ZERO_OFFSET[j])
        return q

    def read_torque(self):
        """返回关节力矩反馈 [abd, hip, knee] (N·m)。"""
        if self.sim:
            return list(self.tau_sim)
        tau = [0.0, 0.0, 0.0]
        for j, m in self.motors.items():
            tau[j] = m.get_motor_torque()  # 已考虑减速比
        return tau

    def send_mit(self, q_kin, vel, kp, kd, ff):
        """运动学角度 → 电机 raw → MIT 指令。"""
        if self.sim:
            for j in range(3):
                # 简单仿真：位置跟随 + 力矩施加
                self.q_sim[j] += 0.3 * (q_kin[j] - self.q_sim[j])
                self.tau_sim[j] = ff
            return
        for j, m in self.motors.items():
            raw = ZERO_OFFSET[j] + JOINT_SIGN[j] * q_kin[j]
            torque_cmd = JOINT_SIGN[j] * ff
            m.motor_mit_cmd(raw, vel, kp, kd, torque_cmd)

    def lock(self):
        for m in self.motors.values():
            m.lock_motor()

    def deinit(self):
        for m in self.motors.values():
            m.lock_motor()
            m.deinit_motor()


# ════════════════════════════════════
# FK/IK 辅助
# ════════════════════════════════════

def ik_standing(kin, dx, dy, z):
    """站立足端目标：髋俯仰轴正下方。"""
    return kin.inverse(vmc.Vec3(dx, dy, z))


def ik_rest(kin, dx, dy, z=-0.15):
    """趴下足端目标：髋俯仰轴正下方，收拢。"""
    return kin.inverse(vmc.Vec3(dx, dy, z))


# ════════════════════════════════════
# S 曲线过渡
# ════════════════════════════════════

def s_curve_to(hw, q_target, duration, kp, kd, label=""):
    """从当前位置 S 曲线过渡到 q_target。"""
    q_start = hw.read()
    dt = SAFE["dt"]
    steps = int(duration / dt)

    for i in range(steps + 1):
        alpha = 0.5 - 0.5 * math.cos(math.pi * (i + 1) / (steps + 1))
        q = [q_start[j] + alpha * (q_target[j] - q_start[j]) for j in range(3)]
        hw.send_mit(q, 0.0, kp, kd, 0.0)
        time.sleep(dt)

        if i % max(1, int(0.2 / dt)) == 0:
            bar = "█" * int(alpha * 20) + "░" * (20 - int(alpha * 20))
            print(f"\r  {label} [{bar}] {alpha*100:3.0f}%  "
                  f"H={q[1]:+6.3f}→{q_target[1]:+6.3f}  "
                  f"K={q[2]:+6.3f}→{q_target[2]:+6.3f}",
                  end="", flush=True)
    print()


# ════════════════════════════════════
# VMC 力控测试
# ════════════════════════════════════

def run_force_control(hw, stand_z, force_target):
    """
    VMC 力控测试循环。

    给定目标足端力 force_target (N)，计算关节力矩 τ = J^T * F，
    通过 MIT 模式施加力矩（kp=0, kd=小值, ff=τ）。

    按 Enter 停止。
    """
    dt = SAFE["dt"]
    kp_f = SAFE["kp_force"]
    kd_f = SAFE["kd_force"]
    max_tau = SAFE["max_torque"]

    foot_stand = vmc.Vec3(hw.dx_s, hw.dy_s, stand_z)

    keep_running = [True]
    def wait_enter():
        input()
        keep_running[0] = False
    threading.Thread(target=wait_enter, daemon=True).start()

    print(f"\nVMC 力控测试:")
    print(f"  目标足端力: Fx={force_target[0]:.1f}N  Fy={force_target[1]:.1f}N  Fz={force_target[2]:.1f}N")
    print(f"  站立 Z={stand_z}m  kp={kp_f}  kd={kd_f}  按 Enter 停止")
    header = (f"{'t':>6s}  {'HIP':>8s}  {'KNEE':>8s}  "
              f"{'τ_H':>8s}  {'τ_K':>8s}  "
              f"{'fx':>8s}  {'fz':>8s}")
    print(header)
    print("-" * 70)

    t = 0.0

    try:
        while keep_running[0]:
            loop_start = time.perf_counter()

            # 1. 读取当前关节角度
            q_act = hw.read()

            # 2. 正运动学得到足端位置
            foot_act = hw.kin.forward(q_act)

            # 3. 计算雅可比并转换足端力为关节力矩
            # τ = J^T * F
            tau_target = hw.kin.foot_force_to_torques(q_act, force_target)

            # 4. 力矩限幅
            tau_cmd = [max(-max_tau, min(max_tau, tau_target[j])) for j in range(3)]

            # 5. 发送力控指令（kp=0, kd=小值, ff=力矩）
            # 位置目标设为当前位置（仅作参考，实际由力矩主导）
            hw.send_mit(q_act, 0.0, kp_f, kd_f, tau_cmd)

            # 6. 读取力矩反馈
            tau_fb = hw.read_torque()

            # 7. 显示（每 100ms）
            if int(t * 10) != int((t - dt) * 10):
                print(f"{t:6.2f}  {q_act[1]:+8.4f}  {q_act[2]:+8.4f}  "
                      f"{tau_fb[1]:+8.3f}  {tau_fb[2]:+8.3f}  "
                      f"{foot_act.x:+8.4f}  {foot_act.z:+8.4f}")

                # 安全检查
                for j, m in hw.motors.items():
                    if abs(tau_fb[j]) > max_tau * 1.2:
                        print(f"  !! {JOINT_NAMES[j]} 力矩 {tau_fb[j]:.1f} 超限，急停！")
                        keep_running[0] = False

            elapsed = time.perf_counter() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
            t += dt

    except KeyboardInterrupt:
        print()
    keep_running[0] = False


# ════════════════════════════════════
# 多力测试
# ════════════════════════════════════

def run_multi_force_test(hw, stand_z, force_list):
    """
    依次测试多个目标力。

    force_list: [(fx, fy, fz, duration), ...]
    """
    dt = SAFE["dt"]
    kp_f = SAFE["kp_force"]
    kd_f = SAFE["kd_force"]
    max_tau = SAFE["max_torque"]

    print(f"\nVMC 多力测试: {len(force_list)} 个目标力")
    print(f"  站立 Z={stand_z}m  kp={kp_f}  kd={kd_f}")
    header = (f"{'t':>6s}  {'phase':>5s}  {'HIP':>8s}  {'KNEE':>8s}  "
              f"{'τ_H':>8s}  {'τ_K':>8s}  "
              f"{'Ftgt_z':>8s}  {'fz':>8s}")
    print(header)
    print("-" * 80)

    t = 0.0
    phase_idx = 0
    phase_start = 0.0

    try:
        while phase_idx < len(force_list):
            loop_start = time.perf_counter()

            # 当前力目标
            fx, fy, fz, duration = force_list[phase_idx]
            force_target = vmc.Vec3(fx, fy, fz)

            # 1. 读取当前关节角度
            q_act = hw.read()

            # 2. 正运动学
            foot_act = hw.kin.forward(q_act)

            # 3. VMC: τ = J^T * F
            tau_target = hw.kin.foot_force_to_torques(q_act, force_target)
            tau_cmd = [max(-max_tau, min(max_tau, tau_target[j])) for j in range(3)]

            # 4. 发送力控指令
            hw.send_mit(q_act, 0.0, kp_f, kd_f, tau_cmd)

            # 5. 读取力矩反馈
            tau_fb = hw.read_torque()

            # 6. 显示（每 100ms）
            if int(t * 10) != int((t - dt) * 10):
                phase_t = t - phase_start
                print(f"{t:6.2f}  {phase_idx:5d}  {q_act[1]:+8.4f}  {q_act[2]:+8.4f}  "
                      f"{tau_fb[1]:+8.3f}  {tau_fb[2]:+8.3f}  "
                      f"{fz:+8.2f}  {foot_act.z:+8.4f}")

                # 安全检查
                for j, m in hw.motors.items():
                    if abs(tau_fb[j]) > max_tau * 1.2:
                        print(f"  !! {JOINT_NAMES[j]} 力矩 {tau_fb[j]:.1f} 超限，急停！")
                        return

            # 7. 相位切换
            if t - phase_start >= duration:
                phase_idx += 1
                phase_start = t
                if phase_idx < len(force_list):
                    print(f"  → 切换至相位 {phase_idx}: "
                          f"Fz={force_list[phase_idx][2]:.1f}N  "
                          f"持续{force_list[phase_idx][3]:.1f}s")

            elapsed = time.perf_counter() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
            t += dt

    except KeyboardInterrupt:
        print()

    print("  ✓ 多力测试完成")


# ════════════════════════════════════
# 主入口
# ════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="单腿 VMC 力控测试")
    parser.add_argument("--leg", type=int, default=1, choices=[0, 1, 2, 3])
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--dx", type=float, default=0.06)
    parser.add_argument("--dy", type=float, default=0.082)
    parser.add_argument("--stand_z", type=float, default=-0.37)
    parser.add_argument("--force_x", type=float, default=0.0, help="目标足端力 X (N)")
    parser.add_argument("--force_y", type=float, default=0.0, help="目标足端力 Y (N)")
    parser.add_argument("--force_z", type=float, default=-15.0, help="目标足端力 Z (N)")
    parser.add_argument("--multi", action="store_true", help="多力测试模式")
    args = parser.parse_args()

    if not args.sim and not HAS_MOTORS:
        print("未找到 motors_py，切换仿真模式")
        args.sim = True

    leg_name = LEG_NAMES[args.leg]
    mode_str = "仿真" if args.sim else "实机"

    print(f"{'='*60}")
    print(f"单腿 VMC 力控测试 — {leg_name} ({mode_str})")
    print(f"  L1=0.2125  L2=0.25025  dx={args.dx}  dy={args.dy}")
    print(f"  站立 Z={args.stand_z}m")
    print(f"{'='*60}")

    if not args.sim:
        input("按 Enter 初始化电机...")

    hw = SingleLegInterface(args.leg, args.sim, args.dx, args.dy)

    # ─── 阶段 0：通信检查 ───
    print(f"\n阶段 0：通信检查")
    q_now = hw.read()
    foot_now = hw.kin.forward(q_now)
    print(f"  {leg_name}: ABD={q_now[0]:+7.4f} HIP={q_now[1]:+7.4f} KNEE={q_now[2]:+7.4f}")
    print(f"  足端: ({foot_now.x:+.4f}, {foot_now.y:+.4f}, {foot_now.z:+.4f})")
    print("  ✓")

    # ─── 阶段 1：起立 ───
    q_stand = ik_standing(hw.kin, hw.dx_s, hw.dy_s, args.stand_z)
    f_stand = hw.kin.forward(q_stand)
    print(f"\n阶段 1：起立（{SAFE['stand_up_time']}s）")
    print(f"  目标: q=({q_stand[0]:+.3f},{q_stand[1]:+.3f},{q_stand[2]:+.3f})  "
          f"足端({f_stand.x:+.3f},{f_stand.y:+.3f},{f_stand.z:+.3f})")
    s_curve_to(hw, q_stand, SAFE["stand_up_time"],
               SAFE["kp_trans"], SAFE["kd_trans"], label="起立")
    print("  ✓ 起立完成")

    # ─── 阶段 2：等待确认 ───
    if not args.sim:
        input("\n按 Enter 开始力控测试...")
    else:
        print("\n(仿真模式，自动继续)")

    # ─── 阶段 3：力控测试 ───
    if args.multi:
        # 多力测试：0N → -10N → -20N → -10N → 0N
        force_list = [
            (0.0, 0.0, 0.0, 2.0),
            (0.0, 0.0, -10.0, 3.0),
            (0.0, 0.0, -20.0, 3.0),
            (0.0, 0.0, -10.0, 3.0),
            (0.0, 0.0, 0.0, 2.0),
        ]
        run_multi_force_test(hw, args.stand_z, force_list)
    else:
        # 单力测试
        force_target = vmc.Vec3(args.force_x, args.force_y, args.force_z)
        run_force_control(hw, args.stand_z, force_target)

    # ─── 阶段 4：趴下 ───
    if not args.sim:
        input("\n按 Enter 趴下...")
    else:
        print("\n(仿真模式，自动继续)")
    q_rest = ik_rest(hw.kin, hw.dx_s, hw.dy_s, -0.15)
    f_rest = hw.kin.forward(q_rest)
    print(f"趴下目标: q=({q_rest[0]:+.3f},{q_rest[1]:+.3f},{q_rest[2]:+.3f})  "
          f"足端({f_rest.x:+.3f},{f_rest.y:+.3f},{f_rest.z:+.3f})")
    s_curve_to(hw, q_rest, 1.5, SAFE["kp_trans"], SAFE["kd_trans"], label="趴下")
    print("  ✓ 趴下完成")

    print("\n卸力...")
    hw.lock()
    hw.deinit()
    print("结束。")


if __name__ == "__main__":
    main()
