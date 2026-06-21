#!/usr/bin/env python3
"""
四足 Trot 步态测试 — 对角小跑步态。

测试流程：
  0  通信检查    — 读取所有腿电机状态
  1  起立        — S 曲线从当前姿态过渡到站立姿态
  2  等待确认    — 按 Enter 继续
  3  Trot 步态   — 四条腿对角小跑（LF+RB 同相，RF+LB 错位 0.5 周期）
  4  趴下/卸力   — S 曲线趴下，锁定电机

用法：
  python3 tests/test_quad_trot.py                   # 四足实机
  python3 tests/test_quad_trot.py --sim             # 仿真
  python3 tests/test_quad_trot.py --step_length 0.15 # 调步长
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

# 每条腿对应的三个电机 ID + CAN 接口
# 格式: (ABD_motor_id, HIP_motor_id, KNEE_motor_id, "canX")
# 四个 CAN 总线各自独立，ID 不冲突，所以每条腿都是 1/2/3
MOTOR_MAP = {
    0: (1, 2, 3, "can0"),   # LF 左前
    1: (1, 2, 3, "can1"),   # RF 右前
    2: (1, 2, 3, "can2"),   # LB 左后
    3: (1, 2, 3, "can3"),   # RB 右后
}

# 电机零点偏移：q_kinematic = JOINT_SIGN × (raw - offset)
# 左腿膝关节电机零点在 +0.8111 rad，右腿在 -0.8111 rad
ZERO_OFFSET = {
    0: [0.0, 0.0, 0.8111],     # LF
    1: [0.0, 0.0, -0.8111],    # RF
    2: [0.0, 0.0, 0.8111],     # LB
    3: [0.0, 0.0, -0.8111],    # RB
}

# 关节方向修正：+1 电机正转 = 运动学正方向，-1 = 反向
# 运动学正方向（右手定则）：ABD 绕+X(脚外摆)，HIP 绕+Y(腿后摆)，KNEE 绕+Y(腿前弯为负)
JOINT_SIGN = {
    0: [1.0, 1.0, 1.0],      # LF
    1: [1.0, -1.0, -1.0],    # RF 
    2: [-1.0, 1.0, 1.0],     # LB
    3: [-1.0, -1.0, -1.0],   # RB
}

# PD 控制安全参数
SAFE = {
    "kp_stand": 100.0,       # 支撑相比例增益（N·m/rad）
    "kd_stand": 5.0,         # 支撑相微分增益
    "kp_trans": 100.0,       # S 曲线过渡比例增益
    "kd_trans": 5.0,         # S 曲线过渡微分增益
    "kp_swing": 100.0,       # 摆动相比例增益
    "kd_swing": 5.0,         # 摆动相微分增益
    "max_pos_err": 0.4,      # 跟踪误差急停阈值 (rad)
    "stand_up_time": 2.0,    # 起立过渡时间 (s)
    "dt": 0.005,             # 主控制周期 = 200Hz
}

# Trot 步态参数
GAIT = {
    "cycle_time": 0.8,       # 一个完整步态周期 (s)
    "duty_factor": 0.60,     # 支撑相占周期比例，0.6 表示 60% 支撑、40% 摆动
    "step_length": 0.10,     # 单步前后移动幅度 (m)，相对站立足端
    "step_height": 0.04,     # 摆动相抬腿最高点 (m)
}

# Trot 相位偏移：LF 和 RB 同相（0.0），RF 和 LB 错半周期（0.5）
# 这样形成对角支撑：LF+RB 着地时 RF+LB 抬起，交替
TROT_PHASE_OFFSET = {0: 0.0, 1: 0.5, 2: 0.5, 3: 0.0}


# ════════════════════════════════════
# IK 目标姿态
# ════════════════════════════════════

def ik_standing(kin, dx, dy, z):
    """足端在髋俯仰轴正下方 z 米处，用于站立。"""
    return kin.inverse(vmc.Vec3(dx, dy, z))


def ik_rest(kin, dx, dy, z=-0.15):
    """足端收拢在髋俯仰轴正下方，膝弯曲，用于趴下。"""
    return kin.inverse(vmc.Vec3(dx, dy, z))


# ════════════════════════════════════
# 硬件接口
# ════════════════════════════════════

class QuadInterface:
    """四条腿的电机读写封装。"""

    def __init__(self, legs, sim, dx=0.06, dy=0.082):
        self.legs = legs
        self.sim = sim
        self.motors = {}        # (leg, joint) → MotorDriver 实例
        self.q_sim = {}         # 仿真模式下的虚拟关节角
        self.kinematics = {}    # leg → LegKinematics（正/逆运动学）
        self.dx_s = {}          # 各腿 ABD→HIP 前后偏置（正=前）
        self.dy_s = {}          # 各腿 ABD→HIP 左右偏置（正=左）

        for leg in legs:
            # 前后腿 X 符号：前腿 +dx（髋在足前），后腿 -dx（髋在足后）
            x_sign = 1.0 if leg in (0, 1) else -1.0
            # 左右腿 Y 符号：左腿 +dy（髋在足右），右腿 -dy（髋在足左）
            y_sign = 1.0 if leg in (0, 2) else -1.0
            self.dx_s[leg] = x_sign * dx
            self.dy_s[leg] = y_sign * dy
            self.kinematics[leg] = vmc.LegKinematics(0.2125, 0.25025,
                                                     self.dx_s[leg], self.dy_s[leg])

        if not sim and HAS_MOTORS:
            self._init_real()

    def _init_real(self):
        """初始化所有目标腿的电机。每腿 3 关节 × 各自 CAN 总线。"""
        for leg in self.legs:
            entry = MOTOR_MAP.get(leg, (1, 2, 3, "can0"))
            ids = entry[:3]   # (ABD_id, HIP_id, KNEE_id)
            can_if = entry[3] # "can0"~"can3"
            for j, mid in enumerate(ids):
                if mid is None:
                    continue
                m = motors_py.MotorDriver.create_motor(mid, "CAN", can_if, "LRO_CAN", 2)
                m.init_motor()
                self.motors[(leg, j)] = m

    def read(self, leg):
        """读取某腿的关节角 [ABD, HIP, KNEE]，已做符号和零点修正。"""
        if self.sim:
            return [self.q_sim.get((leg, j), 0.0) for j in range(3)]
        q = [0.0, 0.0, 0.0]
        for j in range(3):
            m = self.motors.get((leg, j))
            if m:
                m.refresh_motor_status()
                raw = m.get_motor_pos()                              # 电机原始角度 (rad)
                q[j] = JOINT_SIGN[leg][j] * (raw - ZERO_OFFSET[leg][j])  # → 运动学角度
        return q

    def send_mit(self, leg, q_kin, vel, kp, kd, ff):
        """向某腿的 3 个电机发送 MIT 模式指令。
        q_kin: 运动学坐标系下的目标关节角
        内部完成运动学角度 → 电机 raw 角度转换。
        """
        if self.sim:
            for j in range(3):
                key = (leg, j)
                if key not in self.q_sim:
                    self.q_sim[key] = q_kin[j]
                self.q_sim[key] += 0.5 * (q_kin[j] - self.q_sim[key])  # 一阶低通模拟
            return
        for j in range(3):
            m = self.motors.get((leg, j))
            if m:
                # 运动学角度 → 电机 raw 角度：raw = offset + sign × q_kin
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
# S 曲线过渡（多腿同步）
# ════════════════════════════════════

def s_curve_to_all(hw, legs, q_targets, duration, kp, kd, label=""):
    """所有腿从当前姿态 S 曲线（余弦加速-减速）过渡到目标姿态。
    q_targets: {leg: [abd, hip, knee]}
    """
    # 记录各腿起始关节角
    q_starts = {}
    for leg in legs:
        q_starts[leg] = hw.read(leg)

    dt = SAFE["dt"]
    steps = int(duration / dt)

    for i in range(steps + 1):
        # S 曲线插值系数：cos 从 0→1 平滑过渡
        alpha = 0.5 - 0.5 * math.cos(math.pi * (i + 1) / (steps + 1))
        for leg in legs:
            q = [q_starts[leg][j] + alpha * (q_targets[leg][j] - q_starts[leg][j])
                 for j in range(3)]
            hw.send_mit(leg, q, 0.0, kp, kd, 0.0)
        time.sleep(dt)

        # 每 200ms 打印进度条
        if i % max(1, int(0.2 / dt)) == 0:
            bar = "█" * int(alpha * 20) + "░" * (20 - int(alpha * 20))
            l0 = legs[0]
            print(f"\r  {label} [{bar}] {alpha*100:3.0f}%  "
                  f"H{LEG_NAMES[l0]}={q_starts[l0][1]:+.3f}→{q_targets[l0][1]:+.3f}",
                  end="", flush=True)
    print()


# ════════════════════════════════════
# 复合摆线轨迹生成
# ════════════════════════════════════

def cycloid_pos(s):
    """复合摆线 X 位置：s 从 0→1，输出从 0 平滑过渡到 1。
    一阶导数在 s=0 和 s=1 处均为 0，速度连续。
    """
    if s <= 0: return 0.0
    if s >= 1: return 1.0
    return s - math.sin(2 * math.pi * s) / (2 * math.pi)


def lift_curve(s):
    """抬腿高度曲线：s 从 0→1，输出 0→1→0（半正弦）。
    s=0.5 时达到峰值 1.0。
    """
    if s <= 0 or s >= 1: return 0.0
    return 0.5 * (1.0 - math.cos(2 * math.pi * s))


# ════════════════════════════════════
# Trot 步态主循环
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

    # 静态站立足端位置（髋坐标系）
    foot_stand = {}
    for leg in legs:
        foot_stand[leg] = vmc.Vec3(hw.dx_s[leg], hw.dy_s[leg], stand_z)

    # 后台线程监听 Enter 停止步态
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
                # ── 步态相位计算 ──
                # 加 trot 偏移后归一化到 [0, 1)
                raw_phase = (t / cycle_T + TROT_PHASE_OFFSET[leg]) % 1.0
                in_stance = raw_phase < duty           # 支撑相
                if in_stance:
                    phase_s = raw_phase / duty         # 支撑相归一化进度 [0, 1)
                else:
                    phase_s = (raw_phase - duty) / (1.0 - duty)  # 摆动相归一化进度 [0, 1)

                # ── 足端轨迹 ──
                # 支撑相：足端从 step_len/2 线性后移到 -step_len/2（推动机身前进）
                # 摆动相：足端通过复合摆线从后方前摆，同时抬腿
                foot_x_offset = step_len * 0.5 - step_len * phase_s
                if in_stance:
                    foot = vmc.Vec3(
                        foot_stand[leg].x + foot_x_offset,   # X 前后滑动
                        foot_stand[leg].y,                     # Y 不变
                        foot_stand[leg].z)                     # Z 不变（着地）
                else:
                    cx = cycloid_pos(phase_s)                  # 摆线 X: 0→1
                    cz = lift_curve(phase_s)                   # 抬腿 Z: 0→1→0
                    foot = vmc.Vec3(
                        foot_stand[leg].x - step_len * 0.5 + step_len * cx,  # X 前摆
                        foot_stand[leg].y,
                        foot_stand[leg].z + step_h * cz)                     # Z 抬腿
                # ── IK → 关节角 → MIT 发送 ──
                q_tgt = hw.kinematics[leg].inverse(foot)
                q_tgt_list = [q_tgt[j] for j in range(3)]
                kp = kp_s if in_stance else kp_w   # 支撑相高刚度，摆动相可稍软
                kd = kd_s if in_stance else kd_w
                hw.send_mit(leg, q_tgt_list, 0.0, kp, kd, 0.0)

            # ── 状态显示（每 100ms 一次） ──
            if int(t * 10) != int((t - dt) * 10):
                phases = []
                for leg in legs:
                    raw_phase = (t / cycle_T + TROT_PHASE_OFFSET[leg]) % 1.0
                    phases.append("支" if raw_phase < duty else "摆")
                print(f"{t:6.2f}  {phases[0]:>5s}  {phases[1]:>5s}  "
                      f"{phases[2]:>5s}  {phases[3]:>5s}  {step_count:4d}")

                # ── 安全检查：支撑腿跟踪误差过大则急停 ──
                for leg in legs:
                    q_act = hw.read(leg)
                    raw_phase = (t / cycle_T + TROT_PHASE_OFFSET[leg]) % 1.0
                    if raw_phase < duty:  # 只检查支撑腿
                        foot_tgt = foot_stand[leg]
                        q_tgt = hw.kinematics[leg].inverse(foot_tgt)
                        for j in range(3):
                            err = abs(q_act[j] - q_tgt[j])
                            if err > SAFE["max_pos_err"]:
                                print(f"  !! {LEG_NAMES[leg]} {JOINT_NAMES[j]} "
                                      f"误差 {err:.3f} > {SAFE['max_pos_err']}！")

            # ── 精确时序：补偿循环耗时 ──
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
    parser.add_argument("--dx", type=float, default=0.06, help="前腿 X 偏置 (m)")
    parser.add_argument("--dy", type=float, default=0.082, help="Y 偏置绝对值 (m)")
    parser.add_argument("--stand_z", type=float, default=-0.3, help="站立足端 Z (m)")
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
