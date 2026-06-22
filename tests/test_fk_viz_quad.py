#!/usr/bin/env python3
"""
四腿运动学可视化 — 只读模式，调试电机方向符号。

连接四条腿，实时显示各腿连杆动画和足端坐标。
用手搬动单条腿，观察该腿运动方向和角度变化是否与实际一致。

用法：
  python3 tests/test_fk_viz_quad.py              # 四腿实机
  python3 tests/test_fk_viz_quad.py --leg 1      # 只看 RF
  python3 tests/test_fk_viz_quad.py --sim        # 仿真

方向符号配置在下方 JOINT_SIGN 字典中，调试正确后同步到步态测试代码。
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

# ════════════════════════════════════
# 配置
# ════════════════════════════════════
LEG_NAMES = {0: "LF", 1: "RF", 2: "LB", 3: "RB"}

# (ABD, HIP, KNEE, CAN接口) — ABD=None 表示无外展电机
MOTOR_MAP = {
    0: (1, 2, 3, "can0"),   # LF 左前
    1: (1, 2, 3, "can1"),   # RF 右前
    2: (1, 2, 3, "can2"),   # LB 左后
    3: (1, 2, 3, "can3"),   # RB 右后
}

# ── 零点偏移 (rad) ──
ZERO_OFFSET = [0.0, 0.0, -0.8880]   # ABD, HIP, KNEE

# ── 每条腿的电机方向修正 (+1同向, -1反向) ──
# RF 已验证; 其余按镜像推测，通过可视化逐腿验证
JOINT_SIGN = {
    0: [1.0, -1.0, -1.0],   # LF
    1: [1.0, -1.0, -1.0],   # RF — 已验证
    2: [1.0,  1.0, -1.0],   # LB
    3: [1.0,  1.0, -1.0],   # RB
}

# ── 偏置 ──
DX = 0.06
DY = 0.082
L1 = 0.2125
L2 = 0.25025


def get_signed_dx_dy(leg):
    x_sign = 1.0 if leg in (0, 1) else -1.0
    y_sign = 1.0 if leg in (0, 2) else -1.0
    return x_sign * DX, y_sign * DY


# ════════════════════════════════════
# 硬件接口
# ════════════════════════════════════

class QuadFKReader:
    def __init__(self, legs, sim):
        self.legs = legs
        self.sim = sim
        self.motors = {}         # (leg, joint) → motor
        self.q_sim = {}

        if not sim and HAS_MOTORS:
            self._init_real()

    def _init_real(self):
        for leg in self.legs:
            entry = MOTOR_MAP.get(leg, (1, 2, 3, "can0"))
            ids = entry[:3]
            can_if = entry[3]
            printed = False
            for j, mid in enumerate(ids):
                if mid is None:
                    continue
                if not printed:
                    print(f"  {LEG_NAMES[leg]} → {can_if} IDs={ids}")
                    printed = True
                m = motors_py.MotorDriver.create_motor(mid, "CAN", can_if, "LRO_CAN", 2)
                m.init_motor()
                self.motors[(leg, j)] = m

    def read(self, leg):
        if self.sim:
            key = leg
            if key not in self.q_sim:
                self.q_sim[key] = [0.0, 0.0, 0.0]
            return list(self.q_sim[key])
        q = [0.0, 0.0, 0.0]
        for j in range(3):
            m = self.motors.get((leg, j))
            if m:
                m.refresh_motor_status()
                raw = m.get_motor_pos()
                q[j] = JOINT_SIGN[leg][j] * (raw - ZERO_OFFSET[j])
        return q

    def lock_all(self):
        for m in self.motors.values():
            m.lock_motor()

    def deinit_all(self):
        for m in self.motors.values():
            m.lock_motor()
            m.deinit_motor()


# ════════════════════════════════════
# 可视化
# ════════════════════════════════════

class QuadFKVisualizer:
    """四腿 FK 可视化 — 2×2 矢状面 + 关节角/足端坐标显示。"""

    def __init__(self, legs):
        self.legs = legs
        self.kinematics = {}
        for leg in legs:
            dx_s, dy_s = get_signed_dx_dy(leg)
            self.kinematics[leg] = vmc.LegKinematics(L1, L2, dx_s, dy_s)

        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 10))
        self.fig.canvas.manager.set_window_title("四腿 FK 可视化")

        self.leg_positions = [(0, 0), (0, 1), (1, 0), (1, 1)]  # LF, RF, LB, RB
        self.artists = {}

        for leg in legs:
            row, col = self.leg_positions[leg]
            ax = self.axes[row][col]
            dx_s, dy_s = get_signed_dx_dy(leg)
            ax.set_title(f"{LEG_NAMES[leg]}  dx={dx_s:+.3f} dy={dy_s:+.3f}", fontsize=11)
            ax.set_xlabel("X 前→ (m)", fontsize=8)
            ax.set_ylabel("Z 上↑ (m)", fontsize=8)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_xlim(-0.35, 0.35)
            ax.set_ylim(-0.55, 0.10)

            (origin,) = ax.plot([], [], 'ks', ms=6)
            (thigh,) = ax.plot([], [], 'r-', lw=3)
            (calf,) = ax.plot([], [], 'b-', lw=3)
            (knee,) = ax.plot([], [], 'ro', ms=7)
            (foot,) = ax.plot([], [], 'go', ms=9)

            self.artists[leg] = {
                'origin': origin, 'thigh': thigh, 'calf': calf,
                'knee': knee, 'foot': foot,
            }

        # 全局信息文本
        self.info_text = self.fig.text(
            0.02, 0.01, "", fontsize=8, fontfamily='monospace',
            verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.6))

        plt.tight_layout(rect=[0, 0.10, 1, 1])
        plt.ion()
        plt.show()

    def update(self, q_all):
        """q_all: {leg: [abd, hip, knee]}"""
        for leg in self.legs:
            art = self.artists[leg]
            q = q_all.get(leg, [0, 0, 0])
            dx_s, dy_s = get_signed_dx_dy(leg)
            kin = self.kinematics[leg]

            foot_pos = kin.forward(q)
            a, h, k = q[0], q[1], q[2]
            hip_pitch_x = dx_s
            hip_pitch_y = dy_s * math.cos(a)
            hip_pitch_z = dy_s * math.sin(a)

            knee_rel_x = -L1 * math.sin(h)
            knee_rel_z = -L1 * math.cos(h)
            Rx_a = [[1, 0, 0],
                    [0, math.cos(a), -math.sin(a)],
                    [0, math.sin(a), math.cos(a)]]
            knee_x = hip_pitch_x + Rx_a[0][0]*knee_rel_x + Rx_a[0][1]*0 + Rx_a[0][2]*knee_rel_z
            knee_y = hip_pitch_y + Rx_a[1][0]*knee_rel_x + Rx_a[1][1]*0 + Rx_a[1][2]*knee_rel_z
            knee_z = hip_pitch_z + Rx_a[2][0]*knee_rel_x + Rx_a[2][1]*0 + Rx_a[2][2]*knee_rel_z

            foot_rel_x = -L2 * math.sin(h + k)
            foot_rel_z = -L2 * math.cos(h + k)
            foot_x = knee_x + Rx_a[0][0]*foot_rel_x + Rx_a[0][2]*foot_rel_z
            foot_z = knee_z + Rx_a[2][0]*foot_rel_x + Rx_a[2][2]*foot_rel_z

            art['thigh'].set_data([0.0, hip_pitch_x, knee_x],
                                  [0.0, hip_pitch_z, knee_z])
            art['calf'].set_data([knee_x, foot_x], [knee_z, foot_z])
            art['origin'].set_data([0.0], [0.0])
            art['knee'].set_data([knee_x], [knee_z])
            art['foot'].set_data([foot_x], [foot_z])

        # 信息文本
        lines = [" 腿     ABD       HIP      KNEE       fx       fy       fz"]
        for leg in self.legs:
            q = q_all.get(leg, [0, 0, 0])
            foot = self.kinematics[leg].forward(q)
            lines.append(
                f" {LEG_NAMES[leg]}  {q[0]:+7.4f}  {q[1]:+7.4f}  {q[2]:+7.4f}  "
                f"{foot.x:+7.4f} {foot.y:+7.4f} {foot.z:+7.4f}")
        self.info_text.set_text("\n".join(lines))

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def close(self):
        plt.close(self.fig)


# ════════════════════════════════════
# 主入口
# ════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="四腿 FK 可视化 — 调试电机方向")
    parser.add_argument("--leg", type=int, default=None, choices=[0, 1, 2, 3],
                        help="仅显示指定腿 (0=LF 1=RF 2=LB 3=RB)")
    parser.add_argument("--sim", action="store_true")
    args = parser.parse_args()

    if not args.sim and not HAS_MOTORS:
        print("未找到 motors_py，切换仿真模式")
        args.sim = True

    legs = [args.leg] if args.leg is not None else [0, 1, 2, 3]
    leg_str = ", ".join(LEG_NAMES[l] for l in legs)
    mode_str = "仿真" if args.sim else "实机"

    print(f"四腿 FK 可视化 — {leg_str} ({mode_str})")
    print(f"  L1={L1} L2={L2}  dx={DX} dy={DY}")
    print(f"  方向符号: LF={JOINT_SIGN[0]}  RF={JOINT_SIGN[1]}")
    print(f"            LB={JOINT_SIGN[2]}  RB={JOINT_SIGN[3]}")
    print("  搬动腿部，观察足端和角度变化。Ctrl+C 退出。")
    print()

    hw = QuadFKReader(legs, args.sim)
    viz = QuadFKVisualizer(legs)
    t0 = time.time()

    try:
        while True:
            q_all = {}
            for leg in legs:
                if args.sim:
                    t = time.time() - t0
                    phase = t * 0.5 + leg * 0.3
                    q_all[leg] = [0.05 * math.sin(phase * 1.3),
                                  0.4 + 0.3 * math.sin(phase),
                                  -1.2 + 0.2 * math.sin(phase * 0.7)]
                else:
                    q_all[leg] = hw.read(leg)
            viz.update(q_all)
            plt.pause(0.06)

    except KeyboardInterrupt:
        pass
    finally:
        viz.close()
        if not args.sim:
            hw.lock_all()
            hw.deinit_all()
        print("退出。")


if __name__ == "__main__":
    main()
