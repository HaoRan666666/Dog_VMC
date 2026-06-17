#!/usr/bin/env python3
"""
逆运动学验证 — 手柄控制足端位置。

左摇杆：上下=抬踩(Z)  左右=前后(X)
右摇杆：左右=外展(Y)
A 键：使能  |  B 键：停止/卸力

安全措施：
  - 足端范围限位（X±8cm, Y±5cm, Z[-0.35,-0.15]m）
  - kp=20, kd=3（低刚度）
  - 跟踪误差 >0.3rad 急停
  - 无手柄信号 0.5s 直接卸力
  - B 键立即卸力

用法：
  python3 tests/test_ik_joystick.py --leg 2
  python3 tests/test_ik_joystick.py --leg 2 --dx 0.04 --dy -0.03
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Controller', 'build'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Module', 'motors', 'build'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))

import pygame
import numpy as np

from leg_viz import LegVisualizer

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

# ════════════════════════════════
# 安全参数
# ════════════════════════════════
SAFE = {
    "kp": 20.0, "kd": 3.0,
    "max_x": 0.08,    # 足端 X 范围 ±8cm
    "max_y": 0.05,    # 足端 Y 范围 ±5cm
    "z_min": -0.35,   # 足端 Z 下限
    "z_max": -0.15,   # 足端 Z 上限
    "dead_zone": 0.06,
    "cmd_timeout": 0.5,  # 手柄断连超时
    "max_pos_err": 0.3,  # 跟踪误差急停
    "dt": 0.01,          # 100Hz
}
JOINT_NAMES = {0: "ABD", 1: "HIP", 2: "KNEE"}
LEG_NAMES = {0: "LF", 1: "RF", 2: "LB", 3: "RB"}

# 手柄映射
AXIS_LX, AXIS_LY = 0, 1   # 左摇杆
AXIS_RX = 3               # 右摇杆（只用左右）
BTN_A, BTN_B = 0, 1


def dead_zone(val):
    return 0.0 if abs(val) < SAFE["dead_zone"] else val


def main():
    parser = argparse.ArgumentParser(description="IK 手柄控制")
    parser.add_argument("--leg", type=int, default=2, choices=[0, 1, 2, 3])
    parser.add_argument("--dx", type=float, default=0.0)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--viz", action="store_true", help="开启运动学可视化")
    args = parser.parse_args()

    leg_name = LEG_NAMES[args.leg]
    kin = vmc.LegKinematics(0.2125, 0.25025, args.dx, args.dy)

    # ── 电机 ──
    motors = {}
    sim_mode = True
    q_sim = [0.0, 0.0, 0.0]
    if HAS_MOTORS:
        MOTOR_MAP = {0: (1, 2, 3), 1: (4, 5, 6), 2: (7, 8, 9), 3: (10, 11, 12)}
        ids = MOTOR_MAP.get(args.leg, (1, 2, 3))
        try:
            for j, mid in enumerate(ids):
                m = motors_py.MotorDriver.create_motor(mid, "CAN", "can0", "LRO_CAN", 2)
                m.init_motor()
                m.unlock_motor()
                m.set_motor_control_mode(1)
                motors[j] = m
            sim_mode = False
        except RuntimeError as e:
            print(f"无 CAN ({e})，仿真模式")

    mode_str = "仿真" if sim_mode else "实机"
    print(f"IK 手柄控制 — {leg_name} ({mode_str})")
    print(f"  偏置: dx={args.dx:.3f} dy={args.dy:.3f}")
    print(f"  安全: X±{SAFE['max_x']*100:.0f}cm Y±{SAFE['max_y']*100:.0f}cm "
          f"Z[{SAFE['z_min']:.2f},{SAFE['z_max']:.2f}]m")
    # ── pygame 窗口（接收键盘/手柄输入） ──
    pygame.init()
    pygame.joystick.init()
    pygame.display.set_caption("IK 手柄控制 — 点击此窗口再按键")
    pygame.display.set_mode((300, 120))
    js = None
    if pygame.joystick.get_count() > 0:
        js = pygame.joystick.Joystick(0)
        js.init()
        print(f"  手柄: {js.get_name()}")
    else:
        print("  未检测到手柄！WASD=前后/上下  QE=外展  空格=使能")
        print("  !! 请点击 pygame 小窗口获取键盘焦点 !!")
    print(f"  A=使能  B=停止")
    print(f"  左摇杆: 上下=抬踩(Z)  左右=前后(X)")
    print(f"  右摇杆: 左右=外展(Y)")

    # ── 可视化 ──
    viz = None
    if args.viz:
        viz = LegVisualizer(dx=args.dx, dy=args.dy, title=f"IK 手柄控制 — {leg_name}")
        print("  可视化窗口已打开")

    # ── 读取当前足端位置作为起始 ──
    if motors:
        for m in motors.values():
            m.refresh_motor_status()
        q_now = [motors[j].get_motor_pos() for j in range(3)]
    else:
        q_now = [0.0, 0.896, -1.791]
        q_sim = list(q_now)

    foot_now = kin.forward(q_now)
    foot_target = [foot_now.x, foot_now.y, foot_now.z]
    foot_target[0] = max(-SAFE["max_x"], min(SAFE["max_x"], foot_target[0]))
    foot_target[1] = max(-SAFE["max_y"], min(SAFE["max_y"], foot_target[1]))
    foot_target[2] = max(SAFE["z_min"], min(SAFE["z_max"], foot_target[2]))

    enabled = False
    running = True
    last_joy_time = time.time()
    kp, kd = SAFE["kp"], SAFE["kd"]
    dt = SAFE["dt"]
    prev_buttons = [0] * 16

    print(f"\n  起始: foot=({foot_target[0]:+.3f},{foot_target[1]:+.3f},{foot_target[2]:+.3f})")

    header_printed = False

    try:
        while running:
            loop_start = time.perf_counter()

            # ── 手柄事件 ──
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            if js:
                lx = dead_zone(js.get_axis(AXIS_LX))
                ly = dead_zone(js.get_axis(AXIS_LY))
                rx = dead_zone(js.get_axis(AXIS_RX))
                buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
                last_joy_time = time.time()
            else:
                # 键盘模拟
                keys = pygame.key.get_pressed()
                lx = ly = rx = 0.0
                if keys[pygame.K_w]: ly = -1.0
                if keys[pygame.K_s]: ly = 1.0
                if keys[pygame.K_a]: lx = -1.0
                if keys[pygame.K_d]: lx = 1.0
                if keys[pygame.K_q]: rx = -1.0
                if keys[pygame.K_e]: rx = 1.0
                buttons = [0] * 16
                if keys[pygame.K_SPACE]: buttons[BTN_A] = 1
                if keys[pygame.K_b]: buttons[BTN_B] = 1

            # ── 按钮 ──
            if buttons[BTN_A] and not prev_buttons[BTN_A]:
                enabled = True
                print("\n  使能！")
            if buttons[BTN_B] and not prev_buttons[BTN_B]:
                enabled = False
                print("\n  停止。")
            prev_buttons = [b for b in buttons]

            # ── 超时保护：直接卸力 ──
            timed_out = (js and time.time() - last_joy_time > SAFE["cmd_timeout"])
            if timed_out and enabled:
                print("\n  手柄断连，卸力！")
                enabled = False

            if enabled and not timed_out:
                # ── 更新足端目标 ──
                speed_xy = 0.04
                speed_z = 0.02
                foot_target[0] += lx * speed_xy * dt    # 左摇杆左右 → 前后
                foot_target[1] += rx * speed_xy * dt    # 右摇杆左右 → 外展
                foot_target[2] += ly * speed_z * dt     # 左摇杆上下 → 上下

            # 限位
            foot_target[0] = max(-SAFE["max_x"], min(SAFE["max_x"], foot_target[0]))
            foot_target[1] = max(-SAFE["max_y"], min(SAFE["max_y"], foot_target[1]))
            foot_target[2] = max(SAFE["z_min"], min(SAFE["z_max"], foot_target[2]))

            # ── IK ──
            ft = vmc.Vec3(foot_target[0], foot_target[1], foot_target[2])
            q_target = kin.inverse(ft)
            q_target_list = [q_target[j] for j in range(3)]

            # ── 发送电机指令 ──
            if enabled:
                if motors:
                    for j in range(3):
                        motors[j].motor_mit_cmd(q_target_list[j], 0.0, kp, kd, 0.0)
                else:
                    for j in range(3):
                        q_sim[j] += 0.3 * (q_target_list[j] - q_sim[j])
            else:
                # 未使能时保持当前位置，不给力矩
                if motors:
                    state_now = {}
                    for j, m in motors.items():
                        m.refresh_motor_status()
                        pos_now = m.get_motor_pos()
                        m.motor_mit_cmd(pos_now, 0.0, kp, kd, 0.0)

            # ── 显示（每 0.3s） ──
            now = time.time()
            if int(now * 3) != int((now - dt) * 3):
                if motors:
                    for m in motors.values():
                        m.refresh_motor_status()
                    q_actual = [motors[j].get_motor_pos() for j in range(3)]
                else:
                    q_actual = [q_sim[j] for j in range(3)]

                foot_actual = kin.forward(q_actual)
                err = math.sqrt((foot_actual.x - ft.x)**2 +
                                (foot_actual.y - ft.y)**2 +
                                (foot_actual.z - ft.z)**2)

                if not header_printed:
                    hdr = (f"  {'目标X':>8s}{'目标Y':>8s}{'目标Z':>8s}  "
                           f"{'ABD':>8s}{'HIP':>8s}{'KNEE':>8s}  "
                           f"{'实际X':>8s}{'实际Y':>8s}{'实际Z':>8s}  {'误差':>6s}  {'状态'}")
                    print(hdr)
                    header_printed = True

                status = "使能" if enabled else "待命"
                line = (f"  {ft.x:+8.4f}{ft.y:+8.4f}{ft.z:+8.4f}  "
                        f"{q_target_list[0]:+8.4f}{q_target_list[1]:+8.4f}{q_target_list[2]:+8.4f}  "
                        f"{foot_actual.x:+8.4f}{foot_actual.y:+8.4f}{foot_actual.z:+8.4f}  "
                        f"{err:6.4f}  {status}")
                print(line)
                if viz:
                    viz.update(q_actual, q_target_list)

                # 安全检查
                if enabled and motors:
                    for j in range(3):
                        err_j = abs((motors[j].get_motor_pos() if hasattr(motors[j], 'get_motor_pos') else q_actual[j]) - q_target_list[j])
                        if err_j > SAFE["max_pos_err"]:
                            print(f"  !! {JOINT_NAMES[j]} 误差 {err_j:.3f} > {SAFE['max_pos_err']}，急停！")
                            enabled = False

            # 维持循环速率
            elapsed = time.perf_counter() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\n中断。")

    finally:
        # 卸力
        if motors:
            for m in motors.values():
                m.lock_motor()
                m.deinit_motor()
        if viz:
            viz.close()
        pygame.quit()
        print("结束。")


if __name__ == "__main__":
    main()
