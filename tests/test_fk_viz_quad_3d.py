#!/usr/bin/env python3
"""
四腿 3D 运动学可视化 — 含外展关节，调试电机方向符号。

在三维空间中显示四条腿的连杆结构，直观看到外展(左右摆动)、
髋俯仰(前后摆动)、膝弯曲的立体效果。

用法：
  python3 tests/test_fk_viz_quad_3d.py              # 四腿实机
  python3 tests/test_fk_viz_quad_3d.py --sim        # 仿真
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'kinematics', 'build'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Module', 'motors', 'build'))

import matplotlib.pyplot as plt

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
LEG_COLORS = {0: 'orange', 1: 'dodgerblue', 2: 'green', 3: 'red'}

# (ABD, HIP, KNEE, CAN接口)
MOTOR_MAP = {
    0: (1, 2, 3, "can0"),   # LF
    1: (1, 2, 3, "can1"),   # RF
    2: (1, 2, 3, "can2"),   # LB
    3: (1, 2, 3, "can3"),   # RB
}

ZERO_OFFSET = {
    0: [0.0, 0.0, 0.8111],
    1: [0.0, 0.0, -0.8111],
    2: [0.0, 0.0, 0.8111],
    3: [0.0, 0.0, -0.8111],
}


JOINT_SIGN = {
    0: [1.0, 1.0, 1.0],   # LF
    1: [1.0, -1.0, -1.0],   # RF — 已验证
    2: [-1.0,1.0, 1.0],   # LB
    3: [-1.0,-1.0, -1.0],   # RB
}

DX, DY = 0.06, 0.082
L1, L2 = 0.2125, 0.25025

# 机身尺寸
BODY_LENGTH = 0.42   # 前后
BODY_WIDTH = 0.20    # 左右

# 髋在机身坐标系的位置 (x前, y左, z上)
HIP_BODY = {
    0: ( BODY_LENGTH/2,  BODY_WIDTH/2, 0.0),   # LF
    1: ( BODY_LENGTH/2, -BODY_WIDTH/2, 0.0),   # RF
    2: (-BODY_LENGTH/2,  BODY_WIDTH/2, 0.0),   # LB
    3: (-BODY_LENGTH/2, -BODY_WIDTH/2, 0.0),   # RB
}


def fk_points(q, dx_s, dy_s):
    """返回髋坐标系中各点：origin=侧摆轴, offset_x, hip_pitch, knee, foot"""
    a, h, k = q[0], q[1], q[2]
    sa, ca = math.sin(a), math.cos(a)
    sh, ch = math.sin(h), math.cos(h)
    shk, chk = math.sin(h + k), math.cos(h + k)

    # X 方向偏置（纯前后）
    offset_x = (dx_s, 0.0, 0.0)

    # hip_pitch: X + Y 偏置（Y经外展旋转）
    hp = (dx_s, dy_s * ca, dy_s * sa)

    # 大小腿在矢状面内的相对位置
    knee_rel = (-L1 * sh, 0.0, -L1 * ch)
    foot_rel = (-L2 * shk, 0.0, -L2 * chk)

    # 绕 X 旋转 a
    knee = (hp[0] + knee_rel[0],
            hp[1] + ca * knee_rel[1] - sa * knee_rel[2],
            hp[2] + sa * knee_rel[1] + ca * knee_rel[2])

    foot = (knee[0] + foot_rel[0],
            knee[1] + ca * foot_rel[1] - sa * foot_rel[2],
            knee[2] + sa * foot_rel[1] + ca * foot_rel[2])

    origin = (0.0, 0.0, 0.0)
    return origin, offset_x, hp, knee, foot


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
        self.motors = {}
        self.q_sim = {}

        if not sim and HAS_MOTORS:
            self._init_real()

    def _init_real(self):
        for leg in self.legs:
            entry = MOTOR_MAP.get(leg, (1, 2, 3, "can0"))
            ids, can_if = entry[:3], entry[3]
            for j, mid in enumerate(ids):
                if mid is None:
                    continue
                m = motors_py.MotorDriver.create_motor(mid, "CAN", can_if, "LRO_CAN", 2)
                m.init_motor()
                self.motors[(leg, j)] = m
        print(f"  已连接 {len(self.motors)} 个电机")

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
                q[j] = JOINT_SIGN[leg][j] * (raw - ZERO_OFFSET[leg][j])
        return q

    def lock_all(self):
        for m in self.motors.values():
            m.lock_motor()

    def deinit_all(self):
        for m in self.motors.values():
            m.lock_motor()
            m.deinit_motor()


# ════════════════════════════════════
# 3D 可视化
# ════════════════════════════════════

class QuadFKVisualizer3D:
    def __init__(self, legs):
        self.legs = legs
        self.kinematics = {}
        for leg in legs:
            dx_s, dy_s = get_signed_dx_dy(leg)
            self.kinematics[leg] = vmc.LegKinematics(L1, L2, dx_s, dy_s)

        self.fig = plt.figure(figsize=(12, 9))
        self.fig.canvas.manager.set_window_title("四腿 3D FK 可视化")
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.ax.set_xlabel("X 前→ (m)")
        self.ax.set_ylabel("Y 左→ (m)")
        self.ax.set_zlabel("Z 上↑ (m)")
        self.ax.set_xlim(-0.3, 0.35)
        self.ax.set_ylim(-0.25, 0.25)
        self.ax.set_zlim(-0.55, 0.10)
        self.ax.view_init(elev=25, azim=-60)  # 斜视角

        # 机身矩形
        hx, hy = BODY_LENGTH / 2, BODY_WIDTH / 2
        bx = [hx, hx, -hx, -hx, hx]
        by = [hy, -hy, -hy, hy, hy]
        bz = [0, 0, 0, 0, 0]
        self.ax.plot(bx, by, bz, 'k-', lw=2, alpha=0.5)

        # 髋关节点
        for leg in legs:
            hx, hy, hz = HIP_BODY[leg]
            self.ax.plot([hx], [hy], [hz], 'ks', ms=8)

        self.lines = {}
        for leg in legs:
            color = LEG_COLORS[leg]
            # X偏置 (虚线)
            (off_x,) = self.ax.plot([], [], [], '--', lw=1, color='gray', alpha=0.7)
            # Y偏置 (虚线)
            (off_y,) = self.ax.plot([], [], [], '--', lw=1, color='gray', alpha=0.7)
            # 大腿 (实线)
            (thigh,) = self.ax.plot([], [], [], '-', lw=3, color=color,
                                    label=LEG_NAMES[leg])
            (calf,) = self.ax.plot([], [], [], '-', lw=3, color=color)
            (knee,) = self.ax.plot([], [], [], 'o', ms=7, color=color)
            (foot,) = self.ax.plot([], [], [], 'o', ms=10, color=color)
            self.lines[leg] = {'off_x': off_x, 'off_y': off_y,
                               'thigh': thigh, 'calf': calf,
                               'knee': knee, 'foot': foot}
        self.ax.legend(loc='upper left', fontsize=8)

        # 关节角文本
        self.info_text = self.fig.text(
            0.02, 0.02, "", fontsize=8, fontfamily='monospace',
            verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.6))

        plt.ion()
        plt.show()

    def update(self, q_all):
        for leg in self.legs:
            q = q_all.get(leg, [0, 0, 0])
            dx_s, dy_s = get_signed_dx_dy(leg)
            origin, off_x, hp, knee, foot = fk_points(q, dx_s, dy_s)

            # 转换到机身坐标系
            hx_b, hy_b, hz_b = HIP_BODY[leg]
            o_b = (hx_b + origin[0], hy_b + origin[1], hz_b + origin[2])
            ox_b = (hx_b + off_x[0], hy_b + off_x[1], hz_b + off_x[2])
            hp_b = (hx_b + hp[0], hy_b + hp[1], hz_b + hp[2])
            k_b = (hx_b + knee[0], hy_b + knee[1], hz_b + knee[2])
            f_b = (hx_b + foot[0], hy_b + foot[1], hz_b + foot[2])

            art = self.lines[leg]
            # X偏置：origin → offset_x
            art['off_x'].set_data([o_b[0], ox_b[0]], [o_b[1], ox_b[1]])
            art['off_x'].set_3d_properties([o_b[2], ox_b[2]])
            # Y偏置：offset_x → hip_pitch
            art['off_y'].set_data([ox_b[0], hp_b[0]], [ox_b[1], hp_b[1]])
            art['off_y'].set_3d_properties([ox_b[2], hp_b[2]])
            # 大腿：hip_pitch → knee
            art['thigh'].set_data([hp_b[0], k_b[0]], [hp_b[1], k_b[1]])
            art['thigh'].set_3d_properties([hp_b[2], k_b[2]])
            # 小腿：knee → foot
            art['calf'].set_data([k_b[0], f_b[0]], [k_b[1], f_b[1]])
            art['calf'].set_3d_properties([k_b[2], f_b[2]])
            art['knee'].set_data([k_b[0]], [k_b[1]])
            art['knee'].set_3d_properties([k_b[2]])
            art['foot'].set_data([f_b[0]], [f_b[1]])
            art['foot'].set_3d_properties([f_b[2]])

        # 信息文本
        lines = [" 腿     ABD       HIP      KNEE       fx(髋)    fy(髋)    fz(髋)"]
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
    parser = argparse.ArgumentParser(description="四腿 3D FK 可视化")
    parser.add_argument("--sim", action="store_true")
    args = parser.parse_args()

    if not args.sim and not HAS_MOTORS:
        print("未找到 motors_py，切换仿真模式")
        args.sim = True

    legs = [0, 1, 2, 3]
    mode_str = "仿真" if args.sim else "实机"

    print(f"四腿 3D FK 可视化 ({mode_str})")
    print(f"  L1={L1} L2={L2}  dx={DX} dy={DY}")
    print(f"  JOINT_SIGN: LF={JOINT_SIGN[0]} RF={JOINT_SIGN[1]} LB={JOINT_SIGN[2]} RB={JOINT_SIGN[3]}")
    print("  拖动鼠标旋转视角，滚轮缩放。Ctrl+C 退出。")

    hw = QuadFKReader(legs, args.sim)
    viz = QuadFKVisualizer3D(legs)
    t0 = time.time()

    try:
        while True:
            q_all = {}
            for leg in legs:
                if args.sim:
                    t = time.time() - t0
                    phase = t * 0.5 + leg * 0.3
                    q_all[leg] = [0.15 * math.sin(phase * 0.8),
                                  0.3 + 0.25 * math.sin(phase),
                                  -1.2 + 0.15 * math.sin(phase * 0.6)]
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
