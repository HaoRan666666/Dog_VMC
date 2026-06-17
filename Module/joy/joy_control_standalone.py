#!/usr/bin/env python3
"""
四足机器人遥控控制（独立版，不依赖 ROS2，使用 pygame 读取手柄）。

摇杆映射（Xbox 手柄）：
  左摇杆上下 → 前进/后退速度 vx
  左摇杆左右 → 左右平移速度 vy
  右摇杆左右 → 偏航角速度
  右摇杆上下 → 机身高度偏移
  十字键上下 → 步态切换
  A 键 → 使能  |  B 键 → 停止
  X 键 → 站立  |  Y 键 → Trot
  LB  → 减小步高 | RB → 增大步高

用法：
  python3 joy_control_standalone.py
  python3 joy_control_standalone.py --sim
"""

import argparse
import math
import os
import sys
import time
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'motors', 'build'))                # motors
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'Controller', 'build'))      # vmc_controller

try:
    import pygame
except ImportError:
    print("请安装 pygame: pip install pygame")
    sys.exit(1)

try:
    import vmc_controller_py as vmc
except ImportError:
    print("错误：未找到 vmc_controller_py 模块，请先编译 Controller")
    sys.exit(1)

try:
    import motors_py
    HAS_MOTORS = True
except ImportError:
    HAS_MOTORS = False


# ─── 手柄轴/按钮索引（pygame 映射） ───
AXIS_LX = 0
AXIS_LY = 1
AXIS_RX = 3     # 右摇杆左右（偏航角速度）
BTN_A = 0
BTN_B = 1
BTN_X = 2
BTN_Y = 3
BTN_LB = 4
BTN_RB = 5
BTN_START = 7
BTN_BACK = 6

MOTOR_IDS = {
    0: (1, 2, 3),
    1: (4, 5, 6),
    2: (7, 8, 9),
    3: (10, 11, 12),
}

GAIT_NAMES = {vmc.GaitType.TROT: "Trot", vmc.GaitType.STAND: "Stand",
              vmc.GaitType.WALK: "Walk", vmc.GaitType.BOUND: "Bound",
              vmc.GaitType.PACE: "Pace"}
GAIT_ORDER = [vmc.GaitType.STAND, vmc.GaitType.WALK,
              vmc.GaitType.TROT, vmc.GaitType.PACE, vmc.GaitType.BOUND]


class QuadrupedJoyController:
    def __init__(self, sim_mode=False, max_vx=0.3, max_vy=0.15, max_yaw=1.0,
                 dead_zone=0.08, cmd_timeout=0.3):
        self.sim_mode = sim_mode
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_yaw = max_yaw
        self.dead_zone = dead_zone
        self.cmd_timeout = cmd_timeout

        # ─── 控制器 ───
        params = vmc.RobotParams()
        self.ctrl = vmc.QuadrupedController(params)
        self.ctrl.set_config(vmc.GaitType.TROT,
                             step_length=0.06, step_height=0.04,
                             swing_kp=80, swing_kd=5,
                             stance_kp=30, stance_kd=3)

        self.motion = vmc.RobotMotionCommand()
        self.motion.target_euler = vmc.Vec3(0, 0, 0)

        # ─── 状态 ───
        self.enabled = False
        self.current_gait = vmc.GaitType.TROT
        self.step_height = 0.04
        self.last_joy_time = time.time()
        self.prev_buttons = [0] * 16
        self.running = True
        self.joystick = None

        # ─── 电机 ───
        self.motors = {}
        if not sim_mode and HAS_MOTORS:
            self._init_motors()

    def _init_motors(self):
        for leg_idx, (abd_id, hip_id, knee_id) in MOTOR_IDS.items():
            for suffix, mid in [("abd", abd_id), ("hip", hip_id), ("knee", knee_id)]:
                key = f"leg{leg_idx}_{suffix}"
                self.motors[key] = motors_py.MotorDriver.create_motor(
                    mid, "CAN", "can0", "LRO_CAN", 2)
        for motor in self.motors.values():
            motor.init_motor()
            motor.unlock_motor()
            motor.set_motor_control_mode(1)
        print(f"已初始化 {len(self.motors)} 个电机")

    # ─── 手柄处理 ───
    def _dead_zone(self, val):
        return 0.0 if abs(val) < self.dead_zone else val

    def _btn_rising(self, idx, buttons):
        prev = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
        cur = buttons[idx] if idx < len(buttons) else 0
        return cur and not prev

    def process_events(self):
        """处理 pygame 事件队列。返回 True 表示继续，False 表示退出。"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.JOYDEVICEADDED:
                self.joystick = pygame.joystick.Joystick(event.device_index)
                self.joystick.init()
                print(f"检测到手柄: {self.joystick.get_name()}")
            if event.type == pygame.JOYDEVICEREMOVED:
                print("手柄断开")
                self.joystick = None

        if not self.joystick:
            return True

        self.last_joy_time = time.time()
        buttons = [self.joystick.get_button(i)
                   for i in range(self.joystick.get_numbuttons())]

        # ─── 摇杆 → 运动指令 ───
        lx = self._dead_zone(self.joystick.get_axis(AXIS_LX))
        ly = self._dead_zone(self.joystick.get_axis(AXIS_LY))
        rx = self._dead_zone(self.joystick.get_axis(AXIS_RX))
        hat_y = self.joystick.get_hat(0)[1] if self.joystick.get_numhats() > 0 else 0

        self.motion.target_velocity.x = -ly * self.max_vx    # 推向前 = 前进
        self.motion.target_velocity.y = lx * self.max_vy     # 推向左 = 左移
        self.motion.target_yaw_rate = rx * self.max_yaw      # 右摇杆左右 = 偏航

        # 十字键 → 步态切换
        if hat_y > 0:
            self._next_gait()
        elif hat_y < 0:
            self._prev_gait()

        # 按钮
        if self._btn_rising(BTN_A, buttons):
            self._enable()
        if self._btn_rising(BTN_B, buttons):
            self._disable()
        if self._btn_rising(BTN_X, buttons):
            self._set_gait(vmc.GaitType.STAND)
        if self._btn_rising(BTN_Y, buttons):
            self._set_gait(vmc.GaitType.TROT)
        if self._btn_rising(BTN_LB, buttons):
            self._adjust_step_height(-0.005)
        if self._btn_rising(BTN_RB, buttons):
            self._adjust_step_height(0.005)

        self.prev_buttons = buttons
        return True

    # ─── 控制命令 ───
    def _enable(self):
        if self.enabled: return
        print("使能！")
        self.enabled = True
        if not self.sim_mode:
            for m in self.motors.values():
                m.unlock_motor()

    def _disable(self):
        if not self.enabled: return
        print("停止！")
        self.enabled = False
        self.motion.target_velocity.x = 0.0
        self.motion.target_velocity.y = 0.0
        self.motion.target_yaw_rate = 0.0
        if not self.sim_mode:
            for m in self.motors.values():
                m.lock_motor()

    def _set_gait(self, gait_type):
        name = GAIT_NAMES.get(gait_type, "?")
        print(f"步态 → {name}")
        self.current_gait = gait_type
        self.ctrl.set_config(gait_type,
                             step_length=0.06, step_height=self.step_height,
                             swing_kp=80, swing_kd=5,
                             stance_kp=30, stance_kd=3)

    def _next_gait(self):
        try:
            idx = GAIT_ORDER.index(self.current_gait)
        except ValueError:
            idx = 0
        self._set_gait(GAIT_ORDER[(idx + 1) % len(GAIT_ORDER)])

    def _prev_gait(self):
        try:
            idx = GAIT_ORDER.index(self.current_gait)
        except ValueError:
            idx = 0
        self._set_gait(GAIT_ORDER[(idx - 1) % len(GAIT_ORDER)])

    def _adjust_step_height(self, delta):
        self.step_height = max(0.01, min(0.10, self.step_height + delta))
        print(f"步高 = {self.step_height:.3f} m")
        self._set_gait(self.current_gait)

    # ─── 控制循环 ───
    def control_step(self, dt):
        if not self.enabled:
            return

        now = time.time()
        if now - self.last_joy_time > self.cmd_timeout:
            self.motion.target_velocity.x = 0.0
            self.motion.target_velocity.y = 0.0
            self.motion.target_yaw_rate = 0.0

        self.ctrl.set_motion_command(self.motion)

        if self.sim_mode:
            self.ctrl.update_imu(dt, vmc.Vec3(0, 0, -9.81), vmc.Vec3(0, 0, 0))

        commands = self.ctrl.step(dt)

        if not self.sim_mode and self.motors:
            self._send_motor_commands(commands)

    def _send_motor_commands(self, commands):
        for leg_idx, (abd_id, hip_id, knee_id) in MOTOR_IDS.items():
            for j, suffix in enumerate(["abd", "hip", "knee"]):
                cmd = commands.legs[leg_idx][j]
                key = f"leg{leg_idx}_{suffix}"
                if key in self.motors:
                    self.motors[key].motor_mit_cmd(
                        cmd.position, cmd.velocity,
                        cmd.kp, cmd.kd, cmd.feedforward_torque)

    def shutdown(self):
        self.running = False
        self._disable()
        if not self.sim_mode:
            for m in self.motors.values():
                m.deinit_motor()


def main():
    parser = argparse.ArgumentParser(description="四足机器人遥控控制（独立版）")
    parser.add_argument("--sim", action="store_true", help="仿真模式")
    parser.add_argument("--max-vx", type=float, default=0.3)
    parser.add_argument("--max-vy", type=float, default=0.15)
    parser.add_argument("--max-yaw", type=float, default=1.0)
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()
    print(f"检测到 {pygame.joystick.get_count()} 个手柄")

    if pygame.joystick.get_count() == 0:
        print("未检测到手柄，等待连接...")

    controller = QuadrupedJoyController(
        sim_mode=args.sim, max_vx=args.max_vx,
        max_vy=args.max_vy, max_yaw=args.max_yaw)

    dt = 0.005  # 200 Hz
    print(f"控制循环 {1/dt:.0f} Hz，按 Ctrl+C 退出")
    print("A=使能 B=停止 X=站立 Y=Trot LB/RB=调步高 十字键=切换步态")

    try:
        while controller.running:
            loop_start = time.perf_counter()

            if not controller.process_events():
                break

            controller.control_step(dt)

            elapsed = time.perf_counter() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        controller.shutdown()
        pygame.quit()


if __name__ == "__main__":
    main()
