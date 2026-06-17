#!/usr/bin/env python3
"""
四足机器人 VMC 位置控制示例。

演示完整的控制流水线：
  1. 读取 IMU → 状态估计
  2. 生成步态相位 → 复合摆线足端轨迹
  3. 逆运动学 → 目标关节角
  4. VMC 平衡控制器 → 支撑腿力矩前馈
  5. 以 MIT 模式发送指令到电机

用法：
  python example_walk.py [--real] [--duration 10.0]
    --real:     使用真实硬件（电机 + IMU），默认为仿真/空跑模式
    --duration: 运行时长，单位秒
"""

import argparse
import math
import time
import sys
import os

# 将电机模块加入搜索路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'Module', 'motors', 'build'))

# 尝试导入 VMC 控制器的 C++ 绑定
try:
    import vmc_controller_py as vmc
except ImportError:
    print("警告：未找到 vmc_controller_py，将使用 Python 回退实现")
    vmc = None

# 尝试导入电机驱动
try:
    import motors_py
    HAS_MOTORS = True
except ImportError:
    print("警告：未找到 motors_py，将以仿真模式运行")
    HAS_MOTORS = False


# ─── Python 回退：复合摆线函数 ───
def composite_cycloid_pos(s):
    """复合摆线位置曲线：s - sin(2π·s) / (2π)"""
    if s <= 0: return 0.0
    if s >= 1: return 1.0
    return s - math.sin(2 * math.pi * s) / (2 * math.pi)


def composite_cycloid_vel(s):
    """复合摆线速度导数：1 - cos(2π·s)"""
    if s <= 0 or s >= 1: return 0.0
    return 1.0 - math.cos(2 * math.pi * s)


def lift_curve_pos(s):
    """半正弦抬腿曲线：(1 - cos(2π·s)) / 2"""
    if s <= 0 or s >= 1: return 0.0
    return 0.5 * (1.0 - math.cos(2 * math.pi * s))


def lift_curve_vel(s):
    """抬腿曲线速度导数"""
    if s <= 0 or s >= 1: return 0.0
    return math.pi * math.sin(2 * math.pi * s)


# ─── 机器人参数 ───
ROBOT_PARAMS = {
    "body_length": 0.45,         # 机身长度（前后）
    "body_width": 0.25,          # 机身宽度（左右）
    "thigh_length": 0.2125,      # 大腿长度
    "calf_length": 0.25025,      # 小腿长度
    "body_mass": 5.0,            # 机身质量 (kg)
    "nominal_stand_height": 0.25, # 标称站立高度
    "hip_offset_x_front": 0.15,   # 前腿髋 X 偏移
    "hip_offset_x_rear": -0.15,   # 后腿髋 X 偏移
    "hip_offset_y": 0.12,         # 髋 Y 偏移
    "hip_dx": 0.06,              # 侧摆轴→髋俯仰轴 X 偏置 (m)
    "hip_dy": 0.082,             # 侧摆轴→大小腿平面 Y 偏置 (m)
}

# 电机 ID 映射：腿编号 → (侧摆ID, 髋ID, 膝ID)
MOTOR_IDS = {
    vmc.LegIndex.LF if vmc else 0: (1, 2, 3),
    vmc.LegIndex.RF if vmc else 1: (4, 5, 6),
    vmc.LegIndex.LH if vmc else 2: (7, 8, 9),
    vmc.LegIndex.RH if vmc else 3: (10, 11, 12),
}


class QuadrupedSimulator:
    """简易运动学仿真器，用于无硬件时的测试。"""

    def __init__(self, params):
        self.thigh = params.get("thigh_length", 0.2125)
        self.calf = params.get("calf_length", 0.25025)
        self.nominal_height = params.get("nominal_stand_height", 0.25)

        # 各腿关节角 [侧摆, 髋, 膝] — 膝前弯构型
        self.q = [
            [0.0, 0.5, -1.2],   # LF 左前
            [0.0, 0.5, -1.2],   # RF 右前
            [0.0, 0.5, -1.2],   # LB 左后
            [0.0, 0.5, -1.2],   # RB 右后
        ]

    def forward_kinematics(self, leg, q):
        """正向运动学：返回髋坐标系中的足端位置 (x, y, z) — RHR约定"""
        a, h, k = q
        L1, L2 = self.thigh, self.calf
        R = L1 * math.cos(h) + L2 * math.cos(h + k)
        x = -(L1 * math.sin(h) + L2 * math.sin(h + k))  # RHR: 正h → x为负
        y = R * math.sin(a)
        z = -R * math.cos(a)
        return (x, y, z)

    def apply_commands(self, commands):
        """模拟电机响应：将关节角向目标收敛"""
        for leg_idx in range(4):
            cmd = commands.legs[leg_idx] if hasattr(commands, 'legs') else commands[leg_idx]
            for j in range(3):
                target = cmd[j].position if hasattr(cmd[j], 'position') else cmd[j][0]
                self.q[leg_idx][j] += 0.8 * (target - self.q[leg_idx][j])

    def get_joint_state(self):
        """返回控制器期望格式的关节状态"""
        return self.q


def setup_motors():
    """初始化全部 12 个关节的电机驱动"""
    motors = {}
    for leg_idx, (abd_id, hip_id, knee_id) in MOTOR_IDS.items():
        motors[f"leg{leg_idx}_abd"] = motors_py.MotorDriver.create_motor(
            abd_id, "CAN", "can0", "LRO_CAN", 2)
        motors[f"leg{leg_idx}_hip"] = motors_py.MotorDriver.create_motor(
            hip_id, "CAN", "can0", "LRO_CAN", 2)
        motors[f"leg{leg_idx}_knee"] = motors_py.MotorDriver.create_motor(
            knee_id, "CAN", "can0", "LRO_CAN", 2)

    # 初始化全部电机
    for name, motor in motors.items():
        motor.init_motor()
        motor.unlock_motor()
        motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)

    return motors


def send_motor_commands(motors, commands):
    """向全部电机发送 MIT 模式指令"""
    for leg_idx, (abd_id, hip_id, knee_id) in MOTOR_IDS.items():
        cmd_abd = commands.legs[leg_idx][vmc.JointIndex.ABD] if vmc else commands[leg_idx][0]
        cmd_hip = commands.legs[leg_idx][vmc.JointIndex.HIP] if vmc else commands[leg_idx][1]
        cmd_knee = commands.legs[leg_idx][vmc.JointIndex.KNEE] if vmc else commands[leg_idx][2]

        motors[f"leg{leg_idx}_abd"].motor_mit_cmd(
            cmd_abd.position, cmd_abd.velocity,
            cmd_abd.kp, cmd_abd.kd, cmd_abd.feedforward_torque)
        motors[f"leg{leg_idx}_hip"].motor_mit_cmd(
            cmd_hip.position, cmd_hip.velocity,
            cmd_hip.kp, cmd_hip.kd, cmd_hip.feedforward_torque)
        motors[f"leg{leg_idx}_knee"].motor_mit_cmd(
            cmd_knee.position, cmd_knee.velocity,
            cmd_knee.kp, cmd_knee.kd, cmd_knee.feedforward_torque)


def read_joint_state(motors):
    """从电机读取当前关节位置"""
    q = [[0.0, 0.0, 0.0] for _ in range(4)]
    for leg_idx, (abd_id, hip_id, knee_id) in MOTOR_IDS.items():
        motors[f"leg{leg_idx}_abd"].refresh_motor_status()
        motors[f"leg{leg_idx}_hip"].refresh_motor_status()
        motors[f"leg{leg_idx}_knee"].refresh_motor_status()
        q[leg_idx][0] = motors[f"leg{leg_idx}_abd"].get_motor_pos()
        q[leg_idx][1] = motors[f"leg{leg_idx}_hip"].get_motor_pos()
        q[leg_idx][2] = motors[f"leg{leg_idx}_knee"].get_motor_pos()
    return q


def read_imu():
    """读取 IMU 数据。返回 (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)"""
    # 占位：返回仿真静止数据
    return (0.0, 0.0, -9.81, 0.0, 0.0, 0.0)


def print_status(t, body_state, gait_state, motion_cmd):
    """打印控制器运行状态"""
    bs = body_state
    print(f"\r[{t:5.1f}s] v=({motion_cmd.target_velocity.x:+.2f},{motion_cmd.target_velocity.y:+.2f}) "
          f"rpy=({bs.euler.x:+.2f},{bs.euler.y:+.2f},{bs.euler.z:+.2f}) "
          f"cyc={gait_state.cycle_progress:.2f} step={gait_state.step_count}   ", end="")


def main():
    parser = argparse.ArgumentParser(description="四足 VMC 行走示例")
    parser.add_argument("--real", action="store_true", help="使用真实硬件")
    parser.add_argument("--duration", type=float, default=10.0, help="运行时长 (s)")
    parser.add_argument("--vx", type=float, default=0.15, help="前进速度 (m/s)")
    parser.add_argument("--vy", type=float, default=0.0, help="横移速度 (m/s)")
    parser.add_argument("--yaw-rate", type=float, default=0.0, help="偏航角速度 (rad/s)")
    parser.add_argument("--gait", type=str, default="trot",
                        choices=["trot", "walk", "bound", "pace", "stand"],
                        help="步态类型")
    args = parser.parse_args()

    use_real = args.real and HAS_MOTORS

    # ─── 初始化 ───
    motors = None
    if use_real:
        print("正在初始化电机...")
        motors = setup_motors()

    # 创建控制器
    if vmc:
        params = vmc.RobotParams()
        for k, v in ROBOT_PARAMS.items():
            setattr(params, k, v)

        ctrl = vmc.QuadrupedController(params)
        ctrl.set_config(
            getattr(vmc.GaitType, args.gait.upper()),
            0.06, 0.04, 80.0, 5.0, 30.0, 3.0)
    else:
        ctrl = None

    # 设置运动指令
    motion = vmc.RobotMotionCommand() if vmc else type('obj', (object,), {})()
    motion.target_velocity = vmc.Vec3(args.vx, args.vy, 0.0) if vmc else (args.vx, args.vy, 0.0)
    motion.target_yaw_rate = args.yaw_rate
    motion.target_body_height = 0.0
    motion.target_euler = vmc.Vec3(0.0, 0.0, 0.0) if vmc else (0.0, 0.0, 0.0)

    # 仿真器回退
    sim = QuadrupedSimulator(ROBOT_PARAMS)

    # ─── 控制循环 ───
    dt = 0.005  # 200 Hz
    t = 0.0
    print(f"控制循环启动：{1/dt:.0f} Hz，步态={args.gait}，前进速度={args.vx} m/s")
    print("按 Ctrl+C 停止。")

    try:
        while t < args.duration:
            loop_start = time.perf_counter()

            # 1. 读取 IMU
            acc = read_imu()
            accel = vmc.Vec3(acc[0], acc[1], acc[2]) if vmc else acc[:3]
            gyro = vmc.Vec3(acc[3], acc[4], acc[5]) if vmc else acc[3:]

            # 2. 读取关节状态
            if use_real and motors:
                joint_q = read_joint_state(motors)
            else:
                joint_q = sim.get_joint_state()

            # 3. 运行控制器
            if vmc and ctrl:
                ctrl.update_imu(dt, accel, gyro)
                ctrl.set_motion_command(motion)
                commands = ctrl.step(dt)

                # 4. 发送电机指令
                if use_real and motors:
                    send_motor_commands(motors, commands)
                else:
                    sim.apply_commands(commands)

                # 5. 打印状态（每 100ms 一次）
                if int(t * 10) != int((t - dt) * 10):
                    print_status(t, ctrl.body_state, ctrl.gait_state, motion)
            else:
                # 纯 Python 回退控制
                commands = python_control_step(t, dt, sim, motion, args)
                sim.apply_commands(commands)

            # 维持循环速率
            elapsed = time.perf_counter() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

            t += dt

    except KeyboardInterrupt:
        print("\n\n用户停止。")

    finally:
        if use_real and motors:
            print("正在去初始化电机...")
            for name, motor in motors.items():
                motor.lock_motor()
                motor.deinit_motor()

    print("完成。")


def python_control_step(t, dt, sim, motion, args):
    """纯 Python 回退：使用复合摆线轨迹的简化 VMC 控制。"""

    # 步态参数
    cycle_T = 0.30
    duty = 0.50

    # 各腿相位偏移
    gait_offsets = {
        "trot":  [0.0, 0.5, 0.5, 0.0],
        "walk":  [0.0, 0.25, 0.5, 0.75],
        "bound": [0.0, 0.0, 0.5, 0.5],
        "pace":  [0.0, 0.5, 0.0, 0.5],
        "stand": [0.0, 0.0, 0.0, 0.0],
    }
    offsets = gait_offsets.get(args.gait, gait_offsets["trot"])

    step_length = 0.06
    step_height = 0.04

    class SimpleCmd:
        def __init__(self, pos, vel, kp, kd, ff):
            self.position = pos
            self.velocity = vel
            self.kp = kp
            self.kd = kd
            self.feedforward_torque = ff

    commands = [[SimpleCmd(0, 0, 0, 0, 0) for _ in range(3)] for _ in range(4)]

    for leg in range(4):
        # 计算该腿在步态周期中的原始相位
        raw_phase = (t / cycle_T + offsets[leg]) % 1.0

        if raw_phase < duty:
            # 支撑相：足端保持在髋下方
            phase_s = raw_phase / duty
            foot_x = 0.02 - motion.target_velocity.x * dt
            foot_y = ROBOT_PARAMS["hip_offset_y"] * (1 if leg in (0, 2) else -1)
            foot_z = -ROBOT_PARAMS["nominal_stand_height"]
            kp, kd = 30.0, 3.0
            ff_torque = 0.0
        else:
            # 摆动相：复合摆线从后向前
            phase_s = (raw_phase - duty) / (1 - duty)
            cx = composite_cycloid_pos(phase_s)
            cz = lift_curve_pos(phase_s)

            start_x = -step_length * 0.5
            foot_x = start_x + step_length * 1.2 * cx
            foot_y = ROBOT_PARAMS["hip_offset_y"] * (1 if leg in (0, 2) else -1)
            foot_z = -ROBOT_PARAMS["nominal_stand_height"] + step_height * cz
            kp, kd = 80.0, 5.0
            ff_torque = 0.0

        # 逆运动学（与 C++ LegKinematics::inverse 一致，dy=0, dx=0）
        L1, L2 = ROBOT_PARAMS["thigh_length"], ROBOT_PARAMS["calf_length"]

        # 侧摆角
        abd_angle = math.atan2(foot_y, -foot_z) if abs(foot_y) > 0.001 else 0.0

        # YZ 投影距离 R = sqrt(fy² + fz²)，矢状面内 x_leg = fx
        R = math.sqrt(foot_y**2 + foot_z**2)
        x_leg = foot_x

        # 二连杆 IK（矢状面）
        r = math.sqrt(x_leg**2 + R**2)
        r = max(abs(L1 - L2), min(L1 + L2, r))

        cos_k = max(-1.0, min(1.0, (L1**2 + L2**2 - r**2) / (2*L1*L2)))
        knee_angle = -(math.pi - math.acos(cos_k))            # 膝前弯

        beta = math.atan2(x_leg, R)
        cos_a = max(-1.0, min(1.0, (L1**2 + r**2 - L2**2) / (2*L1*r)))
        alpha = math.acos(cos_a)
        hip_angle = alpha - beta                              # 膝前解，RHR绕+y

        commands[leg][0] = SimpleCmd(abd_angle, 0.0, kp, kd, ff_torque)
        commands[leg][1] = SimpleCmd(hip_angle, 0.0, kp, kd, ff_torque)
        commands[leg][2] = SimpleCmd(knee_angle, 0.0, kp, kd, ff_torque)

    return commands


if __name__ == "__main__":
    main()
