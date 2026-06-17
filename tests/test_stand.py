#!/usr/bin/env python3
"""
起立/趴下测试（兼容单腿调试架和四腿实机）。

测试流程：
  0  通信检查    — 读取所有目标腿的电机状态
  1  起立        — S 曲线从当前姿态插值到站立姿态
  2  站立保持    — 保持站立 N 秒
  3  趴下        — S 曲线回到趴下姿态
  4  卸力        — 锁定电机

用法：
  python3 tests/test_stand.py --legs all --sim              # 四腿仿真
  python3 tests/test_stand.py --legs 2 --sim                # 单腿仿真（LB）
  python3 tests/test_stand.py --legs all                    # 四腿实机
  python3 tests/test_stand.py --legs 2 --dx 0.04 --dy -0.03 # 单腿实机带偏置
"""

import argparse
import math
import os
import sys
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
SAFE = {
    "kp_stand": 30.0,      # 站立保持刚度
    "kd_stand": 3.0,
    "kp_trans": 20.0,      # 过渡阶段刚度（更软）
    "kd_trans": 2.0,
    "max_pos_err": 0.4,    # 跟踪误差急停 (rad)
    "stand_up_time": 3.0,  # 起立时间 (s)
    "lie_down_time": 2.5,  # 趴下时间 (s)
    "hold_time": 5.0,      # 站立保持时间 (s)
    "dt": 0.01,            # 控制周期 100Hz
}

LEG_NAMES = {0: "LF", 1: "RF", 2: "LB", 3: "RB"}
JOINT_NAMES = {0: "ABD", 1: "HIP", 2: "KNEE"}
MOTOR_MAP = {0: (1, 2, 3), 1: (None, 1, 2), 2: (7, 8, 9), 3: (10, 11, 12)}

# ── 零点偏移 (rad): q_kinematic = JOINT_SIGN * (q_motor_raw - ZERO_OFFSET) ──
ZERO_OFFSET = [0.0, 0.0, -0.8880]   # ABD, HIP, KNEE: knee ref=-50.88deg

# ── 电机方向修正: +1 同向, -1 反向 ──
JOINT_SIGN = [1.0, -1.0, -1.0]      # ABD, HIP, KNEE


def ik_foot_below_hip(kin, dx, dy, z=-0.25):
    """足端在髋俯仰轴（大腿电机）正下方的关节角。"""
    return kin.inverse(vmc.Vec3(dx, dy, z))


def ik_rest_pose(kin, dx, dy, z=-0.12):
    """趴下姿态：足端收拢在髋俯仰轴正下方，膝大幅弯曲。"""
    return kin.inverse(vmc.Vec3(dx, dy, z))


# ════════════════════════════════════
# 硬件接口（多腿）
# ════════════════════════════════════

class MultiLegInterface:
    def __init__(self, legs, sim, dx=0.0, dy=0.0):
        self.legs = legs           # list of leg indices
        self.sim = sim
        self.motors = {}           # (leg, joint) → motor
        self.q_sim = {}            # (leg, joint) → simulated angle
        self.kinematics = {}

        for leg in legs:
            x_sign = 1.0 if leg in (0, 1) else -1.0
            y_sign = 1.0 if leg in (0, 2) else -1.0
            self.kinematics[leg] = vmc.LegKinematics(0.2125, 0.25025, x_sign * dx, y_sign * dy)

        if not sim and HAS_MOTORS:
            self._init_real()

    def _init_real(self):
        for leg in self.legs:
            ids = MOTOR_MAP.get(leg, (1, 2, 3))
            for j, mid in enumerate(ids):
                if mid is None:
                    continue
                m = motors_py.MotorDriver.create_motor(mid, "CAN", "can0", "LRO_CAN", 2)
                m.init_motor()
                self.motors[(leg, j)] = m

    def read(self, leg):
        state = {}
        for j in range(3):
            key = (leg, j)
            if self.sim:
                if key not in self.q_sim:
                    self.q_sim[key] = 0.0
                state[j] = {"pos": self.q_sim[key]}
            else:
                m = self.motors.get(key)
                if m:
                    m.refresh_motor_status()
                    raw = m.get_motor_pos()
                    state[j] = {"pos": JOINT_SIGN[j] * (raw - ZERO_OFFSET[j])}
                else:
                    state[j] = {"pos": 0.0}
        return state

    def send_mit(self, leg, joint, pos, vel, kp, kd, ff):
        key = (leg, joint)
        if self.sim:
            if key not in self.q_sim:
                self.q_sim[key] = pos
            self.q_sim[key] += 0.5 * (pos - self.q_sim[key])
        else:
            m = self.motors.get(key)
            if m:
                raw = ZERO_OFFSET[joint] + JOINT_SIGN[joint] * pos
                m.motor_mit_cmd(raw, vel, kp, kd, ff)

    def lock_all(self):
        if not self.sim:
            for m in self.motors.values():
                m.lock_motor()

    def deinit_all(self):
        if not self.sim:
            for m in self.motors.values():
                m.lock_motor()
                m.deinit_motor()


# ════════════════════════════════════
# S 曲线过渡
# ════════════════════════════════════

def s_curve_transition(hw, legs, q_targets, duration, kp, kd, label=""):
    """
    所有目标腿从当前姿态 S 曲线过渡到 q_targets。
    q_targets: {leg: [abd, hip, knee]}
    """
    # 读取起始位置
    q_starts = {}
    for leg in legs:
        state = hw.read(leg)
        q_starts[leg] = [state[j]["pos"] for j in range(3)]

    dt = SAFE["dt"]
    steps = int(duration / dt)
    bar_width = 20

    for i in range(steps + 1):
        alpha_s = 0.5 - 0.5 * math.cos(math.pi * (i + 1) / (steps + 1))

        for leg in legs:
            targets = [q_starts[leg][j] + alpha_s * (q_targets[leg][j] - q_starts[leg][j])
                       for j in range(3)]
            for j in range(3):
                hw.send_mit(leg, j, targets[j], 0.0, kp, kd, 0.0)
        time.sleep(dt)

        if i % max(1, int(0.2 / dt)) == 0:
            done = int(alpha_s * bar_width)
            bar = "█" * done + "░" * (bar_width - done)
            # 显示第一条腿的进度
            leg0 = legs[0]
            s = hw.read(leg0)
            line = (f"\r  {label} [{bar}] {alpha_s*100:3.0f}%  "
                    f"H={s[1]['pos']:+6.3f}→{q_targets[leg0][1]:+6.3f}  "
                    f"K={s[2]['pos']:+6.3f}→{q_targets[leg0][2]:+6.3f}")
            print(line, end="", flush=True)
    print()


# ════════════════════════════════════
# 主测试
# ════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="起立/趴下测试")
    parser.add_argument("--legs", type=str, default="all",
                        help="测试腿: 'all' 或 '0,1,2,3' 或单个 '2'")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--dx", type=float, default=0.06)
    parser.add_argument("--dy", type=float, default=0.082)
    parser.add_argument("--stand_z", type=float, default=-0.38,
                        help="站立时足端 Z (负数)，默认 -0.30")
    parser.add_argument("--rest_z", type=float, default=-0.15,
                        help="趴下时足端 Z (负数)，默认 -0.12")
    args = parser.parse_args()

    if not args.sim and not HAS_MOTORS:
        print("未找到 motors_py，切换仿真模式")
        args.sim = True

    # 解析腿列表
    if args.legs == "all":
        legs = [0, 1, 2, 3]
    else:
        legs = [int(x.strip()) for x in args.legs.split(",")]

    leg_names = ", ".join(LEG_NAMES[l] for l in legs)
    n_legs = len(legs)
    print(f"{'='*55}")
    print(f"起立/趴下测试 — {leg_names} ({n_legs}腿)")
    print(f"  模式: {'仿真' if args.sim else '实机'}  dx={args.dx} dy={args.dy}")
    print(f"  站立 Z={args.stand_z}m  趴下 Z={args.rest_z}m")
    print(f"{'='*55}")

    if not args.sim:
        input("按 Enter 开始...")

    hw = MultiLegInterface(legs, args.sim, args.dx, args.dy)

    # 计算各腿目标姿态
    q_stand = {}   # 站立
    q_rest = {}    # 趴下
    for leg in legs:
        x_sign = 1.0 if leg in (0, 1) else -1.0
        y_sign = 1.0 if leg in (0, 2) else -1.0
        dx_s = x_sign * args.dx
        dy_s = y_sign * args.dy
        kin = hw.kinematics[leg]
        q_stand[leg] = ik_foot_below_hip(kin, dx_s, dy_s, args.stand_z)
        q_rest[leg] = ik_rest_pose(kin, dx_s, dy_s, args.rest_z)

    # 打印目标姿态
    print(f"\n目标姿态:")
    for leg in legs:
        sq = q_stand[leg]
        rq = q_rest[leg]
        f_stand = hw.kinematics[leg].forward(sq)
        f_rest = hw.kinematics[leg].forward(rq)
        print(f"  {LEG_NAMES[leg]}: 目标足端({dx_s:+.3f},{dy_s:+.3f},{args.stand_z:+.3f})  "
              f"站立 q=({sq[0]:+.3f},{sq[1]:+.3f},{sq[2]:+.3f})  "
              f"FK({f_stand.x:+.3f},{f_stand.y:+.3f},{f_stand.z:+.3f})")
        print(f"       趴下 q=({rq[0]:+.3f},{rq[1]:+.3f},{rq[2]:+.3f})  "
              f"足端({f_rest.x:+.3f},{f_rest.y:+.3f},{f_rest.z:+.3f})")

    kp_t, kd_t = SAFE["kp_trans"], SAFE["kd_trans"]
    kp_s, kd_s = SAFE["kp_stand"], SAFE["kd_stand"]

    try:
        # ─── 阶段 0：通信检查 ───
        print(f"\n阶段 0：通信检查")
        for leg in legs:
            state = hw.read(leg)
            qs = [f"{state[j]['pos']:+7.4f}" for j in range(3)]
            print(f"  {LEG_NAMES[leg]}: ABD={qs[0]} HIP={qs[1]} KNEE={qs[2]}")
        print("  ✓")

        # ─── 阶段 1：起立 ───
        print(f"\n阶段 1：起立（{SAFE['stand_up_time']}s）")
        s_curve_transition(hw, legs, q_stand, SAFE["stand_up_time"],
                           kp_t, kd_t, label="起立")
        print("  ✓ 起立完成")

        # ─── 阶段 2：站立保持 ───
        print(f"\n阶段 2：站立保持 {SAFE['hold_time']}s ...", end="", flush=True)
        steps = int(SAFE["hold_time"] / SAFE["dt"])
        for i in range(steps):
            for leg in legs:
                tgt = q_stand[leg]
                for j in range(3):
                    hw.send_mit(leg, j, tgt[j], 0.0, kp_s, kd_s, 0.0)
            time.sleep(SAFE["dt"])

            if i % max(1, int(0.5 / SAFE["dt"])) == 0:
                state = hw.read(legs[0])
                err_h = abs(state[1]["pos"] - q_stand[legs[0]][1])
                err_k = abs(state[2]["pos"] - q_stand[legs[0]][2])
                print(f"\r  保持中... HIP_err={err_h:.3f} KNEE_err={err_k:.3f}  ",
                      end="", flush=True)

                if err_h > SAFE["max_pos_err"] or err_k > SAFE["max_pos_err"]:
                    print(f"\n  !! 误差过大，急停！")
                    break
        print("\n  ✓ 站立保持完成")

        # ─── 阶段 3：趴下 ───
        print(f"\n阶段 3：趴下（{SAFE['lie_down_time']}s）")
        s_curve_transition(hw, legs, q_rest, SAFE["lie_down_time"],
                           kp_t, kd_t, label="趴下")
        print("  ✓ 趴下完成")

    except KeyboardInterrupt:
        print("\n用户中断！")

    finally:
        print(f"\n卸力...")
        hw.lock_all()
        hw.deinit_all()
        print("结束。")


if __name__ == "__main__":
    main()
