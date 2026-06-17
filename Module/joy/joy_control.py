#!/usr/bin/env python3
"""
四足机器人遥控控制节点（ROS2 + Joystick）。

摇杆映射（以 Xbox 手柄为例）：
  左摇杆上下 (axis 1) → 前进/后退速度 vx
  左摇杆左右 (axis 0) → 左右平移速度 vy
  右摇杆左右 (axis 3) → 偏航角速度
  右摇杆上下 (axis 4) → 机身高度偏移
  十字键上下 (axis 7) → 步态切换
  A 键 (btn 0)   → 使能/启动
  B 键 (btn 1)   → 停止/卸力
  X 键 (btn 2)   → 站立模式
  Y 键 (btn 3)   → Trot 模式
  LB (btn 4)     → 减小步高
  RB (btn 5)     → 增大步高

用法：
  ros2 run joy joy_node          # 启动手柄驱动节点
  python3 joy_control.py         # 启动本控制节点
  python3 joy_control.py --sim   # 仿真模式（无硬件）
"""

import argparse
import math
import os
import sys
import time

# 将电机模块和控制器模块加入搜索路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'motors', 'build'))                # motors
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'Controller', 'build'))      # vmc_controller

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

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


# ─── 摇杆轴/按钮索引（Xbox 手柄默认映射） ───
AXIS_LX = 0       # 左摇杆左右
AXIS_LY = 1       # 左摇杆上下
AXIS_RX = 3       # 右摇杆左右（偏航角速度）
AXIS_DPAD_Y = 7   # 十字键上下

BTN_A = 0         # 使能
BTN_B = 1         # 停止
BTN_X = 2         # 站立
BTN_Y = 3         # Trot
BTN_LB = 4        # 参数减
BTN_RB = 5        # 参数增


class QuadrupedJoyNode(Node):
    """四足机器人遥控控制 ROS2 节点。"""

    def __init__(self, sim_mode=False):
        super().__init__("quadruped_joy_control")

        # ─── 参数声明 ───
        self.declare_parameter("max_vx", 0.3)
        self.declare_parameter("max_vy", 0.15)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.declare_parameter("dead_zone", 0.08)
        self.declare_parameter("cmd_timeout", 0.3)

        self.max_vx = self.get_parameter("max_vx").value
        self.max_vy = self.get_parameter("max_vy").value
        self.max_yaw_rate = self.get_parameter("max_yaw_rate").value
        self.dead_zone = self.get_parameter("dead_zone").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value

        self.sim_mode = sim_mode

        # ─── 创建控制器 ───
        params = vmc.RobotParams()
        params.thigh_length = 0.20
        params.calf_length = 0.20
        params.body_mass = 5.0
        params.nominal_stand_height = 0.25

        self.ctrl = vmc.QuadrupedController(params)
        self.ctrl.set_config(
            vmc.GaitType.TROT,
            step_length=0.06, step_height=0.04,
            swing_kp=80.0, swing_kd=5.0,
            stance_kp=30.0, stance_kd=3.0,
        )

        # ─── 运动指令 ───
        self.motion = vmc.RobotMotionCommand()
        self.motion.target_velocity = vmc.Vec3(0.0, 0.0, 0.0)
        self.motion.target_yaw_rate = 0.0
        self.motion.target_body_height = 0.0
        self.motion.target_euler = vmc.Vec3(0.0, 0.0, 0.0)
        self.ctrl.set_motion_command(self.motion)

        # ─── 运行时状态 ───
        self.enabled = False
        self.current_gait = vmc.GaitType.TROT
        self.step_height = 0.04
        self.last_joy_time = time.time()
        self.last_btn_state = {}

        # ─── 电机 ───
        self.motors = {}
        if not sim_mode and HAS_MOTORS:
            self._init_motors()

        # ─── ROS2 接口 ───
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )
        self.control_timer = self.create_timer(0.005, self.control_loop)  # 200 Hz

        self.get_logger().info("四足遥控节点已就绪")
        self.get_logger().info(f"  最大前进速度: {self.max_vx} m/s")
        self.get_logger().info(f"  最大横移速度: {self.max_vy} m/s")
        self.get_logger().info(f"  最大偏航角速度: {self.max_yaw_rate} rad/s")
        self.get_logger().info(f"  模式: {'仿真' if sim_mode else '真实硬件'}")
        self.get_logger().info("  按 A 使能 | B 停止 | X 站立 | Y Trot")

    # ─── 电机初始化 ───
    def _init_motors(self):
        MOTOR_IDS = {
            0: (1, 2, 3),    # LF
            1: (4, 5, 6),    # RF
            2: (7, 8, 9),    # LB
            3: (10, 11, 12), # RB
        }
        for leg_idx, (abd_id, hip_id, knee_id) in MOTOR_IDS.items():
            for suffix, mid in [("abd", abd_id), ("hip", hip_id), ("knee", knee_id)]:
                key = f"leg{leg_idx}_{suffix}"
                self.motors[key] = motors_py.MotorDriver.create_motor(
                    mid, "CAN", "can0", "LRO_CAN", 2)

        for name, motor in self.motors.items():
            motor.init_motor()
            motor.unlock_motor()
            motor.set_motor_control_mode(1)  # MIT mode
        self.get_logger().info(f"已初始化 {len(self.motors)} 个电机")

    # ─── 手柄回调 ───
    def joy_callback(self, msg: Joy):
        self.last_joy_time = time.time()

        # ─── 解析摇杆轴 ───
        lx = self._apply_dead_zone(self._get_axis(msg, AXIS_LX))
        ly = self._apply_dead_zone(self._get_axis(msg, AXIS_LY))
        rx = self._apply_dead_zone(self._get_axis(msg, AXIS_RX))

        # 左摇杆上下 → 前进速度（推向前 = 负值 → 正速度）
        self.motion.target_velocity.x = -ly * self.max_vx
        # 左摇杆左右 → 横移速度
        self.motion.target_velocity.y = lx * self.max_vy
        # 右摇杆左右 → 偏航角速度
        self.motion.target_yaw_rate = rx * self.max_yaw_rate

        self.ctrl.set_motion_command(self.motion)

        # ─── 十字键 → 步态切换 ───
        dpad_y = self._get_axis(msg, AXIS_DPAD_Y)
        if dpad_y > 0.5:
            self._next_gait()
        elif dpad_y < -0.5:
            self._prev_gait()

        # ─── 按钮 ───
        buttons = msg.buttons

        # A: 使能
        if self._btn_rising(BTN_A, buttons):
            self._enable()
        # B: 停止
        if self._btn_rising(BTN_B, buttons):
            self._disable()
        # X: 站立
        if self._btn_rising(BTN_X, buttons):
            self._set_gait(vmc.GaitType.STAND)
        # Y: Trot
        if self._btn_rising(BTN_Y, buttons):
            self._set_gait(vmc.GaitType.TROT)
        # LB: 减小步高
        if self._btn_rising(BTN_LB, buttons):
            self._adjust_step_height(-0.005)
        # RB: 增大步高
        if self._btn_rising(BTN_RB, buttons):
            self._adjust_step_height(0.005)

        self.last_btn_state = {i: buttons[i] for i in range(len(buttons))}

    # ─── 控制循环 (200 Hz) ───
    def control_loop(self):
        if not self.enabled:
            return

        dt = 0.005
        now = time.time()

        # 通信超时保护：超过 timeout 无手柄数据则停止
        if now - self.last_joy_time > self.cmd_timeout:
            self.motion.target_velocity.x = 0.0
            self.motion.target_velocity.y = 0.0
            self.motion.target_yaw_rate = 0.0
            self.ctrl.set_motion_command(self.motion)

        # 读取 IMU（仿真模式用假数据）
        if self.sim_mode:
            self.ctrl.update_imu(dt, vmc.Vec3(0, 0, -9.81), vmc.Vec3(0, 0, 0))

        # 执行一步控制
        commands = self.ctrl.step(dt)

        # 发送电机指令
        if not self.sim_mode and self.motors:
            self._send_motor_commands(commands)

    # ─── 辅助方法 ───
    @staticmethod
    def _get_axis(msg, idx):
        return msg.axes[idx] if len(msg.axes) > idx else 0.0

    def _apply_dead_zone(self, val):
        return 0.0 if abs(val) < self.dead_zone else val

    def _btn_rising(self, idx, buttons):
        """检测按钮上升沿（按下瞬间）"""
        if len(buttons) <= idx:
            return False
        prev = self.last_btn_state.get(idx, 0)
        return buttons[idx] == 1 and prev == 0

    def _enable(self):
        if self.enabled:
            return
        self.get_logger().info("使能！机器人开始运动")
        self.enabled = True
        if not self.sim_mode and self.motors:
            for motor in self.motors.values():
                motor.unlock_motor()

    def _disable(self):
        if not self.enabled:
            return
        self.get_logger().info("停止！机器人卸力")
        self.enabled = False
        self.motion.target_velocity = vmc.Vec3(0.0, 0.0, 0.0)
        self.motion.target_yaw_rate = 0.0
        self.ctrl.set_motion_command(self.motion)
        if not self.sim_mode and self.motors:
            for motor in self.motors.values():
                motor.lock_motor()

    def _set_gait(self, gait_type):
        name = {vmc.GaitType.TROT: "Trot", vmc.GaitType.STAND: "Stand",
                vmc.GaitType.WALK: "Walk", vmc.GaitType.BOUND: "Bound",
                vmc.GaitType.PACE: "Pace"}.get(gait_type, "?")
        self.get_logger().info(f"切换步态 → {name}")
        self.current_gait = gait_type
        self.ctrl.set_config(
            gait_type,
            step_length=0.06, step_height=self.step_height,
            swing_kp=80.0, swing_kd=5.0,
            stance_kp=30.0, stance_kd=3.0,
        )

    def _next_gait(self):
        order = [vmc.GaitType.STAND, vmc.GaitType.WALK, vmc.GaitType.TROT,
                 vmc.GaitType.PACE, vmc.GaitType.BOUND]
        idx = order.index(self.current_gait) if self.current_gait in order else 0
        self._set_gait(order[(idx + 1) % len(order)])

    def _prev_gait(self):
        order = [vmc.GaitType.STAND, vmc.GaitType.WALK, vmc.GaitType.TROT,
                 vmc.GaitType.PACE, vmc.GaitType.BOUND]
        idx = order.index(self.current_gait) if self.current_gait in order else 0
        self._set_gait(order[(idx - 1) % len(order)])

    def _adjust_step_height(self, delta):
        self.step_height = max(0.01, min(0.10, self.step_height + delta))
        self.get_logger().info(f"步高 = {self.step_height:.3f} m")
        self._set_gait(self.current_gait)

    def _send_motor_commands(self, commands):
        MOTOR_IDS = {
            0: (1, 2, 3), 1: (4, 5, 6), 2: (7, 8, 9), 3: (10, 11, 12),
        }
        for leg_idx, (abd_id, hip_id, knee_id) in MOTOR_IDS.items():
            for j, suffix in enumerate(["abd", "hip", "knee"]):
                cmd = commands.legs[leg_idx][j]
                key = f"leg{leg_idx}_{suffix}"
                if key in self.motors:
                    self.motors[key].motor_mit_cmd(
                        cmd.position, cmd.velocity,
                        cmd.kp, cmd.kd, cmd.feedforward_torque)

    def shutdown(self):
        self.get_logger().info("正在关闭...")
        self._disable()
        if not self.sim_mode and self.motors:
            for motor in self.motors.values():
                motor.deinit_motor()
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description="四足机器人遥控控制")
    parser.add_argument("--sim", action="store_true", help="仿真模式（无硬件）")
    parser.add_argument("--max-vx", type=float, default=0.3, help="最大前进速度 m/s")
    parser.add_argument("--max-vy", type=float, default=0.15, help="最大横移速度 m/s")
    parser.add_argument("--max-yaw", type=float, default=1.0, help="最大偏航角速度 rad/s")
    args = parser.parse_args()

    rclpy.init(args=sys.argv)

    node = QuadrupedJoyNode(sim_mode=args.sim)
    node.max_vx = args.max_vx
    node.max_vy = args.max_vy
    node.max_yaw_rate = args.max_yaw
    node.get_logger().info(
        f"参数覆盖: max_vx={args.max_vx}, max_vy={args.max_vy}, max_yaw={args.max_yaw}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C 退出...")
    finally:
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
