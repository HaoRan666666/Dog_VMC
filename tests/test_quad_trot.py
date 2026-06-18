#!/usr/bin/env python3
"""
四足 Trot 步态测试。

测试流程：
  0  通信检查    — 读取所有腿电机状态
  1  起立        — S 曲线从当前姿态过渡到站立姿态
  2  等待确认    — 按 Enter 继续
  3  Trot 步态   — 四条腿对角小跑
  4  趴下/卸力   — 按 Enter 趴下，锁定电机

用法：
  python3 tests/test_quad_trot.py                   # 四足实机
  python3 tests/test_quad_trot.py --sim             # 仿真
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
# (ABD, HIP, KNEE) + CAN接口名
MOTOR_MAP = {
    0: (1, 2, 3, "can0"),   # LF 左前
    1: (1, 2, 3, "can1"),   # RF 右前
    2: (1, 2, 3, "can2"),   # LB 左后
    3: (1, 2, 3, "can3"),   # RB 右后
}

# ── 零点偏移 (rad): q_kinematic = JOINT_SIGN * (q_motor_raw - ZERO_OFFSET) ──
ZERO_OFFSET = {
    0: [0.0, 0.0, 0.8111],     # LF
    1: [0.0, 0.0, -0.8111],    # RF
    2: [0.0, 0.0, 0.8111],     # LB
    3: [0.0, 0.0, -0.8111],    # RB
}

JOINT_SIGN = {
    0: [1.0, 1.0, 1.0],      # LF
    1: [1.0, -1.0, -1.0],    # RF — 已验证
    2: [-1.0, 1.0, 1.0],     # LB
    3: [-1.0, -1.0, -1.0],   # RB
}

# ── 安全参数 ──
SAFE = {
    "kp_stand": 100.0,
    "kd_stand": 5.0,
    "kp_trans": 100.0,
    "kd_trans": 5.0,
    "kp_swing": 100.0,
    "kd_swing": 5.0,
    "max_pos_err": 0.4,
    "stand_up_time": 2.0,
    "dt": 0.005,          # 200Hz
}

# ── 步态参数 ──
GAIT = {
    "cycle_time": 0.8,    # 步态周期 (s)
    "duty_factor": 0.60,  # 支撑相占比
    "step_length": 0.10,  # 步长 (m)，四足保守起步
    "step_height": 0.04,  # 抬腿高度 (m)
}

# trot 相位偏移: LF+RB同相, RF+LB同相, 差180°
TROT_PHASE_OFFSET = {0: 0.0, 1: 0.5, 2: 0.5, 3: 0.0}  # LF, RF, LB, RB


# ════════════════════════════════════
# FK/IK 辅助
# ════════════════════════════════════

def ik_standing(kin, dx, dy, z):
    return kin.inverse(vmc.Vec3(dx, dy, z))


def ik_rest(kin, dx, dy, z=-0.15):
    return kin.inverse(vmc.Vec3(dx, dy, z))


# ════════════════════════════════════
# 硬件接口
# ════════════════════════════════════

class QuadInterface:
    def __init__(self, legs, sim, dx=0.06, dy=0.082):
        self.legs = legs
        self.sim = sim
        self.motors = {}        # (leg, joint) → motor
        self.q_sim = {}         # (leg, joint) → simulated angle
        self.kinematics = {}    # leg → LegKinematics
        self.dx_s = {}
        self.dy_s = {}

        for leg in legs:
            x_sign = 1.0 if leg in (0, 1) else -1.0
            y_sign = 1.0 if leg in (0, 2) else -1.0
            self.dx_s[leg] = x_sign * dx
            self.dy_s[leg] = y_sign * dy
            self.kinematics[leg] = vmc.LegKinematics(0.2125, 0.25025,
                                                     self.dx_s[leg], self.dy_s[leg])

        if not sim and HAS_MOTORS:
            self._init_real()

    def _init_real(self):
        for leg in self.legs:
            entry = MOTOR_MAP.get(leg, (1, 2, 3, "can0"))
            ids = entry[:3]
            can_if = entry[3]
            for j, mid in enumerate(ids):
                if mid is None:
                    continue
                m = motors_py.MotorDriver.create_motor(mid, "CAN", can_if, "LRO_CAN", 2)
                m.init_motor()
                self.motors[(leg, j)] = m

    def read(self, leg):
        """返回运动学坐标系下某腿的关节角 [abd, hip, knee]。"""
        if self.sim:
            return [self.q_sim.get((leg, j), 0.0) for j in range(3)]
        q = [0.0, 0.0, 0.0]
        for j in range(3):
            m = self.motors.get((leg, j))
            if m:
                m.refresh_motor_status()
                raw = m.get_motor_pos()
                q[j] = JOINT_SIGN[leg][j] * (raw - ZERO_OFFSET[leg][j])
        return q

    def send_mit(self, leg, q_kin, vel, kp, kd, ff):
        """运动学角度 → 电机 raw → MIT 指令。"""
        if self.sim:
            for j in range(3):
                key = (leg, j)
                if key not in self.q_sim:
                    self.q_sim[key] = q_kin[j]
                self.q_sim[key] += 0.5 * (q_kin[j] - self.q_sim[key])
            return
        for j in range(3):
            m = self.motors.get((leg, j))
            if m:
                raw = ZERO_OFFSET[leg][j] + JOINT_SIGN[leg][j] * q_kin[j]
                m.motor_mit_cmd(raw, vel, kp, kd, ff)

    def lock_all(self):
        for m in self.motors.values():
            m.lock_motor()

    def deinit_all(self):
        for m in self.motors.values():
            m.lock_motor()
            m.deinit_motor()


# ════════════════════════════════════
# S 曲线过渡
# ════════════════════════════════════

def s_curve_to_all(hw, legs, q_targets, duration, kp, kd, label=""):
    """所有腿从当前位置 S 曲线过渡到 q_targets。"""
    q_starts = {}
    for leg in legs:
        q_starts[leg] = hw.read(leg)
    dt = SAFE["dt"]
    steps = int(duration / dt)

    for i in range(steps + 1):
        alpha = 0.5 - 0.5 * math.cos(math.pi * (i + 1) / (steps + 1))
        for leg in legs:
            q = [q_starts[leg][j] + alpha * (q_targets[leg][j] - q_starts[leg][j])
                 for j in range(3)]
            hw.send_mit(leg, q, 0.0, kp, kd, 0.0)
        time.sleep(dt)

        if i % max(1, int(0.2 / dt)) == 0:
            bar = "█" * int(alpha * 20) + "░" * (20 - int(alpha * 20))
            l0 = legs[0]
            print(f"\r  {label} [{bar}] {alpha*100:3.0f}%  "
                  f"H{LEG_NAMES[l0]}={q_starts[l0][1]:+.3f}→{q_targets[l0][1]:+.3f}",
                  end="", flush=True)
    print()


# ════════════════════════════════════
# 复合摆线轨迹
# ════════════════════════════════════

def cycloid_pos(s):
    if s <= 0: return 0.0
    if s >= 1: return 1.0
    return s - math.sin(2 * math.pi * s) / (2 * math.pi)


def lift_curve(s):
    if s <= 0 or s >= 1: return 0.0
    return 0.5 * (1.0 - math.cos(2 * math.pi * s))


# ════════════════════════════════════
# Trot 步态
# ════════════════════════════════════

def run_quad_trot(hw, legs, stand_z):
    """四足 Trot 步态控制循环。按 Enter 停止。"""
    dt = SAFE["dt"]
    cycle_T = GAIT["cycle_time"]
    duty = GAIT["duty_factor"]
    step_len = GAIT["step_length"]
    step_h = GAIT["step_height"]

    kp_s, kd_s = SAFE["kp_stand"], SAFE["kd_stand"]
    kp_w, kd_w = SAFE["kp_swing"], SAFE["kd_swing"]

    # 各腿站立足端
    foot_stand = {}
    for leg in legs:
        foot_stand[leg] = vmc.Vec3(hw.dx_s[leg], hw.dy_s[leg], stand_z)

    keep_running = [True]
    def wait_enter():
        input()
        keep_running[0] = False
    threading.Thread(target=wait_enter, daemon=True).start()

    t = 0.0
    step_count = 0

    print(f"\nTrot 步态: 周期={cycle_T}s  duty={duty}  步长={step_len}m  抬腿={step_h}m")
    print(f"  站立 Z={stand_z}m  按 Enter 停止")
    header = (f"{'t':>6s}  {'LF':>6s}  {'RF':>6s}  {'LB':>6s}  {'RB':>6s}  {'步':>4s}")
    print(header)
    print("-" * 55)

    try:
        while keep_running[0]:
            loop_start = time.perf_counter()

            for leg in legs:
                # 步态相位（带 trot 偏移）
                raw_phase = (t / cycle_T + TROT_PHASE_OFFSET[leg]) % 1.0
                in_stance = raw_phase < duty
                if in_stance:
                    phase_s = raw_phase / duty
                else:
                    phase_s = (raw_phase - duty) / (1.0 - duty)

                # 足端目标
                foot_x_offset = step_len * 0.5 - step_len * phase_s
                if in_stance:
                    foot = vmc.Vec3(
                        foot_stand[leg].x + foot_x_offset,
                        foot_stand[leg].y,
                        foot_stand[leg].z)
                else:
                    cx = cycloid_pos(phase_s)
                    cz = lift_curve(phase_s)
                    foot = vmc.Vec3(
                        foot_stand[leg].x - step_len * 0.5 + step_len * cx,
                        foot_stand[leg].y,
                        foot_stand[leg].z + step_h * cz)

                # IK
                q_tgt = hw.kinematics[leg].inverse(foot)
                q_tgt_list = [q_tgt[j] for j in range(3)]

                # 发送
                kp = kp_s if in_stance else kp_w
                kd = kd_s if in_stance else kd_w
                hw.send_mit(leg, q_tgt_list, 0.0, kp, kd, 0.0)

            # 显示（每 100ms）
            if int(t * 10) != int((t - dt) * 10):
                phases = []
                for leg in legs:
                    raw_phase = (t / cycle_T + TROT_PHASE_OFFSET[leg]) % 1.0
                    phases.append("支" if raw_phase < duty else "摆")
                print(f"{t:6.2f}  {phases[0]:>5s}  {phases[1]:>5s}  "
                      f"{phases[2]:>5s}  {phases[3]:>5s}  {step_count:4d}")

                # 安全检查
                for leg in legs:
                    q_act = hw.read(leg)
                    foot_tgt = foot_stand[leg]  # 简化：只检查站立腿
                    raw_phase = (t / cycle_T + TROT_PHASE_OFFSET[leg]) % 1.0
                    if raw_phase < duty:
                        q_tgt = hw.kinematics[leg].inverse(foot_tgt)
                        for j in range(3):
                            err = abs(q_act[j] - q_tgt[j])
                            if err > SAFE["max_pos_err"]:
                                print(f"  !! {LEG_NAMES[leg]} {JOINT_NAMES[j]} 误差 {err:.3f} > {SAFE['max_pos_err']}！")

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
    parser = argparse.ArgumentParser(description="四足 Trot 步态测试")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--dx", type=float, default=0.06)
    parser.add_argument("--dy", type=float, default=0.082)
    parser.add_argument("--stand_z", type=float, default=-0.3)
    args = parser.parse_args()

    if not args.sim and not HAS_MOTORS:
        print("未找到 motors_py，切换仿真模式")
        args.sim = True

    mode_str = "仿真" if args.sim else "实机"
    legs = [0, 1, 2, 3]

    print(f"{'='*60}")
    print(f"四足 Trot 步态测试 ({mode_str})")
    print(f"  L1=0.2125  L2=0.25025  dx={args.dx}  dy={args.dy}")
    print(f"  站立 Z={args.stand_z}m  步长={GAIT['step_length']:.3f}m")
    print(f"  MOTOR: ABD=1 HIP=2 KNEE=3 (每条腿)")
    print(f"{'='*60}")

    if not args.sim:
        input("按 Enter 初始化电机...")

    hw = QuadInterface(legs, args.sim, args.dx, args.dy)

    # ─── 阶段 0：通信检查 ───
    print(f"\n阶段 0：通信检查")
    for leg in legs:
        q_now = hw.read(leg)
        foot_now = hw.kinematics[leg].forward(q_now)
        print(f"  {LEG_NAMES[leg]}: ABD={q_now[0]:+7.4f} HIP={q_now[1]:+7.4f} KNEE={q_now[2]:+7.4f}")
        print(f"       足端({foot_now.x:+.4f}, {foot_now.y:+.4f}, {foot_now.z:+.4f})")
    print("  ✓")

    # ─── 阶段 1：起立 ───
    q_stand_targets = {}
    for leg in legs:
        q_stand_targets[leg] = ik_standing(hw.kinematics[leg],
                                           hw.dx_s[leg], hw.dy_s[leg], args.stand_z)
    f0 = hw.kinematics[0].forward(q_stand_targets[0])
    print(f"\n阶段 1：起立（{SAFE['stand_up_time']}s）")
    print(f"  LF 目标: 足端({f0.x:+.3f},{f0.y:+.3f},{f0.z:+.3f})")
    s_curve_to_all(hw, legs, q_stand_targets, SAFE["stand_up_time"],
                   SAFE["kp_trans"], SAFE["kd_trans"], label="起立")
    print("  ✓ 起立完成")

    # ─── 阶段 2：等待确认 ───
    if not args.sim:
        input("\n按 Enter 开始 Trot 步态...")
    else:
        print("\n(仿真模式，自动继续)")

    # ─── 阶段 3：Trot 步态 ───
    run_quad_trot(hw, legs, args.stand_z)

    # ─── 阶段 4：趴下 ───
    print()
    q_rest_targets = {}
    for leg in legs:
        q_rest_targets[leg] = ik_rest(hw.kinematics[leg],
                                      hw.dx_s[leg], hw.dy_s[leg], -0.15)
    print("趴下...")
    s_curve_to_all(hw, legs, q_rest_targets, 1.5,
                   SAFE["kp_trans"], SAFE["kd_trans"], label="趴下")
    print("  ✓ 趴下完成")

    print("\n卸力...")
    hw.lock_all()
    hw.deinit_all()
    print("结束。")


if __name__ == "__main__":
    main()
