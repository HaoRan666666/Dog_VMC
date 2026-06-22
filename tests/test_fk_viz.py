#!/usr/bin/env python3
"""
单腿运动学可视化 — 只读模式。

不控制电机，仅读取关节角度，FK 计算足端位置，实时显示连杆动画。
用手搬动腿部，观察连杆和足端坐标变化。

用法：
  python3 tests/test_fk_viz.py                         # 默认 RF

零点修改：编辑下方 ZERO_OFFSET 数组，单位 rad。
  q_kinematic = JOINT_SIGN * (q_motor_raw - ZERO_OFFSET)
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'kinematics', 'build'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Module', 'motors', 'build'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))

import matplotlib.pyplot as plt
from leg_viz import LegVisualizer

try:
    import kinematics_py as vmc
except ImportError:
    print("请先编译 Controller")
    sys.exit(1)

HAS_MOTORS = False
try:
    import motors_py
    HAS_MOTORS = True
except (ImportError, RuntimeError):
    pass

LEG_NAMES = {0: "LF", 1: "RF", 2: "LB", 3: "RB"}

# ── 零点偏移 (rad)：q_kinematic = JOINT_SIGN * (q_motor_raw - ZERO_OFFSET) ──
ZERO_OFFSET = [0.0, 0.0, -0.8111]   # ABD, HIP, KNEE: knee ref=-50.88deg at raw=0

# ── 电机方向修正：+1 同向，-1 反向 ──
JOINT_SIGN = [-1.0, -1.0, -1.0]       # ABD, HIP, KNEE


def main():
    parser = argparse.ArgumentParser(description="单腿 FK 可视化")
    parser.add_argument("--leg", type=int, default=3, choices=[0, 1, 2, 3])
    parser.add_argument("--dx", type=float, default=0.06)
    parser.add_argument("--dy", type=float, default=0.082)
    args = parser.parse_args()

    leg_name = LEG_NAMES[args.leg]

    x_sign = 1.0 if args.leg in (0, 1) else -1.0
    y_sign = 1.0 if args.leg in (0, 2) else -1.0
    dx_signed = x_sign * args.dx
    dy_signed = y_sign * args.dy

    motors = {}
    sim_mode = True
    # RF: ABD=None, HIP=CAN1, KNEE=CAN2
    MOTOR_MAP = {0: (1, 2, 3), 1: (1, 2, 3), 2: (1, 2, 3), 3: (1, 2, 3)}
    ids = MOTOR_MAP.get(args.leg, (None, 1, 2))
    if HAS_MOTORS:
        try:
            for j, mid in enumerate(ids):
                if mid is None:
                    continue
                m = motors_py.MotorDriver.create_motor(mid, "CAN", "can3", "LRO_CAN", 2)
                m.init_motor()
                motors[j] = m
            sim_mode = False
        except RuntimeError:
            pass

    mode_str = "实机" if not sim_mode else "模拟"
    print(f"单腿 FK 可视化 — {leg_name} ({mode_str})  dx={dx_signed:.3f} dy={dy_signed:.3f}")
    if any(abs(o) > 1e-6 for o in ZERO_OFFSET):
        print(f"  零点偏移: ABD={ZERO_OFFSET[0]:+.4f} HIP={ZERO_OFFSET[1]:+.4f} KNEE={ZERO_OFFSET[2]:+.4f}")
    print("  Ctrl+C 退出")

    viz = LegVisualizer(dx=dx_signed, dy=dy_signed, title=f"单腿 FK — {leg_name}")
    t0 = time.time()

    try:
        while True:
            if motors:
                for m in motors.values():
                    m.refresh_motor_status()
                q = [0.0, 0.0, 0.0]
                for j in motors:
                    q[j] = JOINT_SIGN[j] * (motors[j].get_motor_pos() - ZERO_OFFSET[j])
            else:
                t = time.time() - t0
                q = [0.1 * math.sin(t * 0.7),
                     0.5 + 0.4 * math.sin(t * 0.5),
                     -1.5 + 0.3 * math.sin(t * 0.3)]

            viz.update(q)
            plt.pause(0.08)

    except KeyboardInterrupt:
        pass
    finally:
        viz.close()
        print("退出。")


if __name__ == "__main__":
    main()
