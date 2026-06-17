#!/usr/bin/env python3
"""
单腿 Trot 步态测试。

测试流程：
  0  通信检查    — 读取电机状态
  1  起立        — S 曲线从当前姿态过渡到站立姿态
  2  等待确认    — 按 Enter 继续
  3  Trot 步态   — 摆动相抬腿前伸、支撑相保持站立
  4  卸力        — 锁定电机

用法：
  python3 tests/test_trot_single.py                   # RF 实机
  python3 tests/test_trot_single.py --leg 1           # 指定腿
  python3 tests/test_trot_single.py --sim             # 仿真
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
    "kp_swing": 120.0,
    "kd_swing": 5.0,
    "max_pos_err": 0.4,
    "stand_up_time": 2.0,
    "dt": 0.005,          # 200Hz
}

# ── 步态参数 ──
GAIT = {
    "cycle_time": 0.8,   # 步态周期 (s)
    "duty_factor": 0.60,  # 支撑相占比
    "step_length": 0.25,  # 步长 (m)
    "step_height": 0.05,  # 抬腿高度 (m)
}


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
# 硬件接口（同 test_stand.py）
# ════════════════════════════════════

class SingleLegInterface:
    def __init__(self, leg, sim, dx=0.06, dy=0.082):
        self.leg = leg
        self.sim = sim
        self.motors = {}
        self.q_sim = [0.0, 0.0, 0.0]

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

    def send_mit(self, q_kin, vel, kp, kd, ff):
        """运动学角度 → 电机 raw → MIT 指令。"""
        if self.sim:
            for j in range(3):
                self.q_sim[j] += 0.5 * (q_kin[j] - self.q_sim[j])
            return
        for j, m in self.motors.items():
            raw = ZERO_OFFSET[j] + JOINT_SIGN[j] * q_kin[j]
            m.motor_mit_cmd(raw, vel, kp, kd, ff)

    def lock(self):
        for m in self.motors.values():
            m.lock_motor()

    def deinit(self):
        for m in self.motors.values():
            m.lock_motor()
            m.deinit_motor()


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
# 复合摆线轨迹
# ════════════════════════════════════

def cycloid_pos(s):
    """复合摆线位置: s - sin(2π·s)/(2π), s∈[0,1]"""
    if s <= 0: return 0.0
    if s >= 1: return 1.0
    return s - math.sin(2 * math.pi * s) / (2 * math.pi)


def lift_curve(s):
    """半正弦抬腿: (1 - cos(2π·s))/2, s∈[0,1]"""
    if s <= 0 or s >= 1: return 0.0
    return 0.5 * (1.0 - math.cos(2 * math.pi * s))


# ════════════════════════════════════
# Trot 步态
# ════════════════════════════════════

def run_trot(hw, stand_z):
    """单腿 Trot 步态控制循环。按 Enter 停止。"""
    dt = SAFE["dt"]
    cycle_T = GAIT["cycle_time"]
    duty = GAIT["duty_factor"]
    step_len = GAIT["step_length"]
    step_h = GAIT["step_height"]

    kp_s, kd_s = SAFE["kp_stand"], SAFE["kd_stand"]
    kp_w, kd_w = SAFE["kp_swing"], SAFE["kd_swing"]

    foot_stand = vmc.Vec3(hw.dx_s, hw.dy_s, stand_z)

    keep_running = [True]
    def wait_enter():
        input()
        keep_running[0] = False
    threading.Thread(target=wait_enter, daemon=True).start()

    t = cycle_T * duty * 0.5  # 从支撑相中间起步，和站立姿态平滑衔接
    step_count = 0

    print(f"\nTrot 步态: 周期={cycle_T}s  duty={duty}  步长={step_len}m  抬腿={step_h}m")
    print(f"  站立 Z={stand_z}m  按 Enter 停止")
    header = (f"{'t':>6s}  {'phase':>6s}  {'进度':>6s}  "
              f"{'HIP':>8s}  {'KNEE':>8s}  "
              f"{'fx':>8s}  {'fz':>8s}  {'步':>4s}")
    print(header)
    print("-" * 70)

    try:
        while keep_running[0]:
            loop_start = time.perf_counter()

            # 1. 步态相位
            raw_phase = (t / cycle_T) % 1.0
            in_stance = raw_phase < duty
            if in_stance:
                phase_s = raw_phase / duty
            else:
                phase_s = (raw_phase - duty) / (1.0 - duty)

            # 2. 足端目标
            # 支撑相：足端从前向后匀速移动（模拟机身前进）
            # 摆动相：足端从后向前摆线运动
            foot_x_offset = step_len * 0.5 - step_len * phase_s
            if in_stance:
                foot = vmc.Vec3(
                    foot_stand.x + foot_x_offset,
                    foot_stand.y,
                    foot_stand.z)
            else:
                cx = cycloid_pos(phase_s)
                cz = lift_curve(phase_s)
                foot = vmc.Vec3(
                    foot_stand.x - step_len * 0.5 + step_len * cx,
                    foot_stand.y,
                    foot_stand.z + step_h * cz)

            # 3. IK
            q_tgt = hw.kin.inverse(foot)
            q_tgt_list = [q_tgt[j] for j in range(3)]

            # 4. 发送指令
            kp = kp_s if in_stance else kp_w
            kd = kd_s if in_stance else kd_w
            hw.send_mit(q_tgt_list, 0.0, kp, kd, 0.0)

            # 5. 显示（每 100ms）
            if int(t * 10) != int((t - dt) * 10):
                q_act = hw.read()
                f_act = hw.kin.forward(q_act)
                phase_name = "支" if in_stance else "摆"
                print(f"{t:6.2f}  {phase_name:>5s}  {phase_s:6.3f}  "
                      f"{q_act[1]:+8.4f}  {q_act[2]:+8.4f}  "
                      f"{f_act.x:+8.4f}  {f_act.z:+8.4f}  {step_count:4d}")

                for j, m in hw.motors.items():
                    err = abs(q_act[j] - q_tgt_list[j])
                    if err > SAFE["max_pos_err"]:
                        print(f"  !! {JOINT_NAMES[j]} 误差 {err:.3f} > {SAFE['max_pos_err']}，急停！")
                        keep_running[0] = False

            elapsed = time.perf_counter() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
            t += dt
            step_count = int(t / cycle_T)

    except KeyboardInterrupt:
        print()
    keep_running[0] = False


# ════════════════════════════════════
# 主入口
# ════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="单腿 Trot 步态测试")
    parser.add_argument("--leg", type=int, default=1, choices=[0, 1, 2, 3])
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--dx", type=float, default=0.06)
    parser.add_argument("--dy", type=float, default=0.082)
    parser.add_argument("--stand_z", type=float, default=-0.37)
    args = parser.parse_args()

    if not args.sim and not HAS_MOTORS:
        print("未找到 motors_py，切换仿真模式")
        args.sim = True

    leg_name = LEG_NAMES[args.leg]
    mode_str = "仿真" if args.sim else "实机"

    print(f"{'='*60}")
    print(f"单腿 Trot 步态测试 — {leg_name} ({mode_str})")
    print(f"  L1=0.2125  L2=0.25025  dx={args.dx}  dy={args.dy}")
    print(f"  站立 Z={args.stand_z}m  步长={GAIT['step_length']:.3f}m")
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
        input("\n按 Enter 开始 Trot 步态...")
    else:
        print("\n(仿真模式，自动继续)")

    # ─── 阶段 3：Trot 步态 ───
    run_trot(hw, args.stand_z)

    # ─── 阶段 4：趴下（ Enter 触发）───
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
