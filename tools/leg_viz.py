#!/usr/bin/env python3
"""
单腿运动学可视化模块。

可在任意脚本中导入使用：
    from leg_viz import LegVisualizer
    viz = LegVisualizer(dx=0.04, dy=-0.03, L1=0.20, L2=0.20)
    viz.update(q)  # q = [abd, hip, knee]

也可以独立运行演示：
    python3 tests/leg_viz.py
"""

import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# ── 中文字体配置 ──
_CN_FONT = None
try:
    for name in ['Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'SimHei',
                 'AR PL UMing CN', 'Noto Sans SC', 'Source Han Sans SC']:
        try:
            matplotlib.font_manager.findfont(name, fallback_to_default=False)
            _CN_FONT = name
            break
        except Exception:
            continue
except Exception:
    pass

if _CN_FONT:
    matplotlib.rcParams['font.family'] = [_CN_FONT, 'sans-serif']
    matplotlib.rcParams['axes.unicode_minus'] = False

# 根据是否有中文字体选择标签文字
def _L(cn, en):
    return cn if _CN_FONT else en


def compute_joints(q, dx=0.06, dy=0.082, L1=0.2125, L2=0.25025):
    """FK 计算关键点：侧摆轴、髋俯仰轴、膝、足端。"""
    a, h, k = q[0], q[1], q[2]
    hip_pitch = np.array([dx, dy * math.cos(a), dy * math.sin(a)])
    knee_rel = np.array([-L1 * math.sin(h), 0, -L1 * math.cos(h)])
    Rx_a = np.array([[1, 0, 0],
                     [0, math.cos(a), -math.sin(a)],
                     [0, math.sin(a), math.cos(a)]])
    knee = hip_pitch + Rx_a @ knee_rel
    foot_rel = np.array([-L2 * math.sin(h + k), 0, -L2 * math.cos(h + k)])
    foot = knee + Rx_a @ foot_rel
    origin = np.array([0.0, 0.0, 0.0])
    return origin, hip_pitch, knee, foot


class LegVisualizer:
    """单腿运动学实时可视化。"""

    def __init__(self, dx=0.06, dy=0.082, L1=0.2125, L2=0.25025, title="单腿 FK"):
        self.dx, self.dy = dx, dy
        self.L1, self.L2 = L1, L2

        self.fig, (self.ax_xz, self.ax_yz) = plt.subplots(1, 2, figsize=(10, 5))
        self.fig.canvas.manager.set_window_title(title)

        for ax, t, xl, yl in [
            (self.ax_xz, _L("矢状面 XZ（侧视）", "Sagittal XZ (side)"), _L("X 前→ (m)", "X fwd (m)"), _L("Z 上↑ (m)", "Z up (m)")),
            (self.ax_yz, _L("冠状面 YZ（正视）", "Coronal YZ (front)"), _L("Y 左→ (m)", "Y left (m)"), _L("Z 上↑ (m)", "Z up (m)")),
        ]:
            ax.set_title(t, fontsize=11)
            ax.set_xlabel(xl)
            ax.set_ylabel(yl)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_xlim(-0.5, 0.5)
            ax.set_ylim(-0.5, 0.2)

        # ── 实际（实心） ──
        (self.origin_xz,) = self.ax_xz.plot([], [], 'ks', ms=6, label=_L('侧摆轴', 'Abd axis'))
        (self.thigh_xz,)  = self.ax_xz.plot([], [], 'r-', lw=3, label=_L(f'大腿 L1={L1}', f'Thigh L1={L1}'))
        (self.calf_xz,)   = self.ax_xz.plot([], [], 'b-', lw=3, label=_L(f'小腿 L2={L2}', f'Calf L2={L2}'))
        (self.knee_xz,)   = self.ax_xz.plot([], [], 'ro', ms=8, label=_L('膝(实)', 'Knee act'))
        (self.foot_xz,)   = self.ax_xz.plot([], [], 'go', ms=10, label=_L('足端(实)', 'Foot act'))

        # ── 目标（空心） ──
        (self.thigh_tgt_xz,) = self.ax_xz.plot([], [], 'r--', lw=1.5, alpha=0.6)
        (self.calf_tgt_xz,)  = self.ax_xz.plot([], [], 'b--', lw=1.5, alpha=0.6)
        (self.knee_tgt_xz,)  = self.ax_xz.plot([], [], 'o', ms=8, mfc='none', mec='red', alpha=0.6,
                                                label=_L('膝(目标)', 'Knee tgt'))
        (self.foot_tgt_xz,)  = self.ax_xz.plot([], [], 'o', ms=10, mfc='none', mec='green', alpha=0.6,
                                                label=_L('足端(目标)', 'Foot tgt'))

        (self.origin_yz,) = self.ax_yz.plot([], [], 'ks', ms=6)
        (self.thigh_yz,)  = self.ax_yz.plot([], [], 'r-', lw=3)
        (self.calf_yz,)   = self.ax_yz.plot([], [], 'b-', lw=3)
        (self.knee_yz,)   = self.ax_yz.plot([], [], 'ro', ms=8)
        (self.foot_yz,)   = self.ax_yz.plot([], [], 'go', ms=10)

        (self.thigh_tgt_yz,) = self.ax_yz.plot([], [], 'r--', lw=1.5, alpha=0.6)
        (self.calf_tgt_yz,)  = self.ax_yz.plot([], [], 'b--', lw=1.5, alpha=0.6)
        (self.knee_tgt_yz,)  = self.ax_yz.plot([], [], 'o', ms=8, mfc='none', mec='red', alpha=0.6)
        (self.foot_tgt_yz,)  = self.ax_yz.plot([], [], 'o', ms=10, mfc='none', mec='green', alpha=0.6)

        self.ax_xz.legend(loc='upper right', fontsize=8)

        self.info = self.ax_xz.text(
            0.02, 0.98, "", transform=self.ax_xz.transAxes,
            fontsize=9, fontfamily='monospace', verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.ion()
        plt.show()

    def update(self, q_actual, q_target=None):
        """q_actual = [abd, hip, knee] 实际值。q_target 可选，目标值。"""
        o, hp, knee, foot = compute_joints(q_actual, self.dx, self.dy, self.L1, self.L2)

        # ── 实际（实线实心） ──
        self.thigh_xz.set_data([o[0], hp[0], knee[0]], [o[2], hp[2], knee[2]])
        self.calf_xz.set_data([knee[0], foot[0]], [knee[2], foot[2]])
        self.origin_xz.set_data([o[0]], [o[2]])
        self.knee_xz.set_data([knee[0]], [knee[2]])
        self.foot_xz.set_data([foot[0]], [foot[2]])

        self.thigh_yz.set_data([o[1], hp[1], knee[1]], [o[2], hp[2], knee[2]])
        self.calf_yz.set_data([knee[1], foot[1]], [knee[2], foot[2]])
        self.origin_yz.set_data([o[1]], [o[2]])
        self.knee_yz.set_data([knee[1]], [knee[2]])
        self.foot_yz.set_data([foot[1]], [foot[2]])

        # ── 目标（虚线空心） ──
        if q_target is not None:
            ot, hpt, knee_t, foot_t = compute_joints(q_target, self.dx, self.dy, self.L1, self.L2)

            self.thigh_tgt_xz.set_data([ot[0], hpt[0], knee_t[0]], [ot[2], hpt[2], knee_t[2]])
            self.calf_tgt_xz.set_data([knee_t[0], foot_t[0]], [knee_t[2], foot_t[2]])
            self.knee_tgt_xz.set_data([knee_t[0]], [knee_t[2]])
            self.foot_tgt_xz.set_data([foot_t[0]], [foot_t[2]])

            self.thigh_tgt_yz.set_data([ot[1], hpt[1], knee_t[1]], [ot[2], hpt[2], knee_t[2]])
            self.calf_tgt_yz.set_data([knee_t[1], foot_t[1]], [knee_t[2], foot_t[2]])
            self.knee_tgt_yz.set_data([knee_t[1]], [knee_t[2]])
            self.foot_tgt_yz.set_data([foot_t[1]], [foot_t[2]])

            err = math.sqrt((foot[0]-foot_t[0])**2 + (foot[1]-foot_t[1])**2 + (foot[2]-foot_t[2])**2)
        else:
            err = None

        dist = math.sqrt(foot[0]**2 + foot[1]**2 + foot[2]**2)
        info = (_L("── 实际 ──\n", "-- Actual --\n") +
                f"ABD={q_actual[0]:+7.4f} rad\n"
                f"HIP={q_actual[1]:+7.4f} rad\n"
                f"KNEE={q_actual[2]:+7.4f} rad\n"
                f"fx={foot[0]:+.4f} m  fy={foot[1]:+.4f} m\n"
                f"fz={foot[2]:+.4f} m  d={dist:.4f} m")
        if err is not None:
            info += (_L("\n── 目标 ──\n", "\n-- Target --\n") +
                     f"ABD={q_target[0]:+7.4f} rad\n"
                     f"HIP={q_target[1]:+7.4f} rad\n"
                     f"KNEE={q_target[2]:+7.4f} rad\n"
                     f"fx={foot_t[0]:+.4f} m  fy={foot_t[1]:+.4f} m\n"
                     f"fz={foot_t[2]:+.4f} m\n"
                     f"Δ={err:.4f} m")
        self.info.set_text(info)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def close(self):
        plt.close(self.fig)


# ═══════════════════════════════════════════
# 四腿步态可视化（2×2 矢状面）
# ═══════════════════════════════════════════

LEG_NAMES_CN = {0: _L("左前 LF", "LF"), 1: _L("右前 RF", "RF"),
                2: _L("左后 LB", "LB"), 3: _L("右后 RB", "RB")}
STANCE_COLOR = '#2ecc71'   # 支撑相 — 绿色
SWING_COLOR = '#3498db'    # 摆动相 — 蓝色


class GaitVisualizer:
    """四腿步态实时可视化 — 2×2 矢状面 (XZ) 视图。"""

    def __init__(self, dx=0.06, dy=0.082, L1=0.2125, L2=0.25025,
                 title=_L("四足步态可视化", "Quadruped Gait Viz")):
        self.dx, self.dy = dx, dy
        self.L1, self.L2 = L1, L2

        self.fig, self.axes = plt.subplots(2, 2, figsize=(10, 9))
        self.fig.canvas.manager.set_window_title(title)

        # LF=左上, RF=右上, LB=左下, RB=右下
        self.leg_positions = [(0, 0), (0, 1), (1, 0), (1, 1)]

        self.artists = {}  # leg → dict of plot handles

        for leg in range(4):
            row, col = self.leg_positions[leg]
            ax = self.axes[row][col]
            ax.set_title(LEG_NAMES_CN.get(leg, f"Leg{leg}"), fontsize=12)
            ax.set_xlabel(_L("X 前→ (m)", "X fwd (m)"), fontsize=8)
            ax.set_ylabel(_L("Z 上↑ (m)", "Z up (m)"), fontsize=8)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_xlim(-0.35, 0.35)
            ax.set_ylim(-0.50, 0.10)

            (origin,) = ax.plot([], [], 'ks', ms=6)
            (thigh,) = ax.plot([], [], '-', lw=3, color=STANCE_COLOR)
            (calf,) = ax.plot([], [], '-', lw=3, color=STANCE_COLOR)
            (knee,) = ax.plot([], [], 'o', ms=7, color=STANCE_COLOR)
            (foot_actual,) = ax.plot([], [], 'o', ms=9, color=STANCE_COLOR)
            (thigh_tgt,) = ax.plot([], [], '--', lw=1.5, alpha=0.5, color='gray')
            (calf_tgt,) = ax.plot([], [], '--', lw=1.5, alpha=0.5, color='gray')
            (foot_tgt,) = ax.plot([], [], 'o', ms=9, mfc='none', mec='gray', alpha=0.5)

            self.artists[leg] = {
                'origin': origin, 'thigh': thigh, 'calf': calf,
                'knee': knee, 'foot_actual': foot_actual,
                'thigh_tgt': thigh_tgt, 'calf_tgt': calf_tgt, 'foot_tgt': foot_tgt,
            }

            # 图例（仅在第一个子图上显示）
            if leg == 0:
                from matplotlib.lines import Line2D
                legend_elements = [
                    Line2D([0], [0], color=STANCE_COLOR, lw=3, label=_L('支撑相', 'Stance')),
                    Line2D([0], [0], color=SWING_COLOR, lw=3, label=_L('摆动相', 'Swing')),
                    Line2D([0], [0], marker='o', color='w', markerfacecolor=STANCE_COLOR,
                           markersize=9, label=_L('足端(实际)', 'Foot act')),
                    Line2D([0], [0], marker='o', color='w', markerfacecolor='none',
                           markeredgecolor='gray', markersize=9, label=_L('足端(目标)', 'Foot tgt')),
                ]
                ax.legend(handles=legend_elements, loc='upper right', fontsize=7)

        self.info_text = self.fig.text(
            0.02, 0.02, "", fontsize=9, fontfamily='monospace',
            verticalalignment='bottom',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.6))

        plt.tight_layout(rect=[0, 0.06, 1, 1])
        plt.ion()
        plt.show()

    def update(self, q_actual, q_targets, phases, t, cycle_progress, step_count):
        """
        q_actual:  [4][3] 实际关节角
        q_targets: [4][3] 目标关节角（可选，传 None 则隐藏）
        phases:    [4] LegPhase 枚举 (0=STANCE, 1=SWING)
        """
        for leg in range(4):
            art = self.artists[leg]
            phase = phases[leg]  # 0=STANCE, 1=SWING
            color = STANCE_COLOR if phase == 0 else SWING_COLOR

            o, hp, knee, foot = compute_joints(
                q_actual[leg], self.dx, self.dy, self.L1, self.L2)

            # 实际姿态（实线）
            art['thigh'].set_data([o[0], hp[0], knee[0]], [o[2], hp[2], knee[2]])
            art['thigh'].set_color(color)
            art['calf'].set_data([knee[0], foot[0]], [knee[2], foot[2]])
            art['calf'].set_color(color)
            art['origin'].set_data([o[0]], [o[2]])
            art['knee'].set_data([knee[0]], [knee[2]])
            art['knee'].set_color(color)
            art['foot_actual'].set_data([foot[0]], [foot[2]])
            art['foot_actual'].set_color(color)

            # 目标姿态（虚线空心）
            if q_targets is not None:
                ot, hpt, knee_t, foot_t = compute_joints(
                    q_targets[leg], self.dx, self.dy, self.L1, self.L2)
                art['thigh_tgt'].set_data([ot[0], hpt[0], knee_t[0]], [ot[2], hpt[2], knee_t[2]])
                art['calf_tgt'].set_data([knee_t[0], foot_t[0]], [knee_t[2], foot_t[2]])
                art['foot_tgt'].set_data([foot_t[0]], [foot_t[2]])
            else:
                art['thigh_tgt'].set_data([], [])
                art['calf_tgt'].set_data([], [])
                art['foot_tgt'].set_data([], [])

        # 状态文本
        phase_names = [_L("支", "S"), _L("摆", "W")]
        pstr = " ".join(f"{LEG_NAMES_CN[l][:2]}:{phase_names[phases[l]]}"
                       for l in range(4))
        info = (f"t={t:5.1f}s  cycle={cycle_progress:.2f}  step={step_count}\n"
                f"{pstr}")
        self.info_text.set_text(info)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def close(self):
        plt.close(self.fig)


# ═══════════════════════════════════════════
# 独立演示
# ═══════════════════════════════════════════
if __name__ == "__main__":
    import time
    print("单腿可视化演示 — 正弦波模拟")
    viz = LegVisualizer()
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            q = [0.1 * math.sin(t * 0.7),
                 0.5 + 0.4 * math.sin(t * 0.5),
                 -1.5 + 0.3 * math.sin(t * 0.3)]
            viz.update(q)
            plt.pause(0.08)
    except KeyboardInterrupt:
        viz.close()
        print("退出。")
