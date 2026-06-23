#!/usr/bin/env python3
"""
四足机器人遥控控制节点（ROS2 + Joystick + IK 位控）。

不依赖 VMC 控制器，直接使用 IK → 关节角 → MIT 位置控制。

操作逻辑：
  RB (btn 5)     → 使能（S 曲线过渡到趴下姿态） / 再次按下 → 失能（急停）
  B  (btn 1)     → 站立 / 趴下 切换
  A  (btn 0)     → 站立 / Trot 步态 切换
  RT (axis 5)    → 站立姿态下升高机身
  LT (axis 2)    → 站立姿态下降低机身
  左摇杆 (axis 0/1)   → Trot 下平移（步长映射）
  右摇杆左右 (axis 3)  → Trot 下旋转

用法：
  ros2 run joy joy_node          # 启动手柄驱动
  python3 joy_control.py         # 启动本控制节点
  python3 joy_control.py --sim   # 仿真模式
"""

import argparse
import enum
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Module', 'motors', 'build'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'kinematics', 'build'))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

try:
    import kinematics_py as vmc
except ImportError:
    print("错误：未找到 kinematics_py 模块，请先编译 kinematics")
    sys.exit(1)

try:
    import motors_py
    HAS_MOTORS = True
except ImportError:
    HAS_MOTORS = False


# ═══════════════════════════════════════════════════════
# 手柄轴/按钮索引（Xbox + ROS2 joy_node 默认映射）
# ═══════════════════════════════════════════════════════
AXIS_LX = 0       # 左摇杆左右
AXIS_LY = 1       # 左摇杆上下
AXIS_LT = 2       # 左扳机（降低高度）
AXIS_RX = 3       # 右摇杆左右（偏航角速度）
AXIS_RT = 5       # 右扳机（升高高度）
AXIS_DPAD_Y = 7   # 十字键上下（调整抬腿高度）

BTN_A  = 0        # 站立 / Trot 切换
BTN_B  = 1        # 站立 / 趴下 切换
BTN_RB = 5        # 使能 / 失能


# ═══════════════════════════════════════════════════════
# 机器人参数（与电机零点、运动学相关）
# ═══════════════════════════════════════════════════════
LEG_NAMES  = {0: "LF", 1: "RF", 2: "LB", 3: "RB"}
JOINT_NAMES = {0: "ABD", 1: "HIP", 2: "KNEE"}

# 每条腿 (ABD_id, HIP_id, KNEE_id, CAN接口)
MOTOR_MAP = {
    0: (1, 2, 3, "can0"),   # LF 左前
    1: (1, 2, 3, "can1"),   # RF 右前
    2: (1, 2, 3, "can2"),   # LB 左后
    3: (1, 2, 3, "can3"),   # RB 右后
}

# 电机零点偏移: q_kinematic = JOINT_SIGN × (raw - offset)
ZERO_OFFSET = {
    0: [0.0, 0.0,  0.8111],     # LF
    1: [0.0, 0.0, -0.8111],     # RF
    2: [0.0, 0.0,  0.8111],     # LB
    3: [0.0, 0.0, -0.8111],     # RB
}

# 关节方向修正: +1 同向, -1 反向
# 运动学正方向: ABD 绕+X(脚外摆), HIP 绕+Y(腿后摆), KNEE 绕+Y(腿前弯为负)
JOINT_SIGN = {
    0: [ 1.0,  1.0,  1.0],      # LF
    1: [ 1.0, -1.0, -1.0],      # RF
    2: [-1.0,  1.0,  1.0],      # LB
    3: [-1.0, -1.0, -1.0],      # RB
}

# 运动学参数（与 test_quad_trot 一致）
L_THIGH = 0.2125    # 大腿长度 (m)
L_CALF  = 0.25025   # 小腿长度 (m)
HIP_DX  = 0.06      # ABD→HIP 前后偏置 (绝对值)
HIP_DY  = 0.082     # ABD→HIP 左右偏置 (绝对值)

# 足端 X 微调（叠加到 dx_s 上，负→更靠后，正→更靠前）
FOOT_X_OFFSET = {0: 0.0, 1: 0.0, 2: -0.03, 3: -0.03}


# 机身几何（COM 系）
BODY_LENGTH = 0.42  # 前后髋间距 (m)
BODY_WIDTH  = 0.154 # 左右髋间距 (m)

# 髋关节 COM 坐标
HIP_COM_X = {0:  BODY_LENGTH / 2, 1:  BODY_LENGTH / 2,
             2: -BODY_LENGTH / 2, 3: -BODY_LENGTH / 2}
HIP_COM_Y = {0:  BODY_WIDTH / 2,  1: -BODY_WIDTH / 2,
             2:  BODY_WIDTH / 2,  3: -BODY_WIDTH / 2}

# PD 控制参数
KP_STAND  = 100.0   # 站立/支撑相刚度
KD_STAND  = 5.0
KP_TRANS  = 100.0   # S 曲线过渡刚度
KD_TRANS  = 5.0
KP_SWING  = 100.0   # 摆动相刚度
KD_SWING  = 5.0


# ══════════════════════════════════════════════════════r═
# 高度参数（足端 Z，相对髋关节，负=向下）
# ═══════════════════════════════════════════════════════
CROUCH_FOOT_Z       = -0.15   # 趴下足端 Z（对应身高 ~0.15m）
DEFAULT_STAND_FOOT_Z = -0.25  # 默认站立足端 Z（对应身高 ~0.25m）
MAX_STAND_FOOT_Z    = -0.38   # 最高站立足端 Z（对应身高 ~0.38m）
HEIGHT_SMOOTH_SPEED = 0.03    # 平滑过渡速度 (m/s)


# ═══════════════════════════════════════════════════════
# Trot 步态参数
# ═══════════════════════════════════════════════════════
TROT_CYCLE_TIME  = 0.8        # 完整步态周期 (s)
TROT_DUTY_FACTOR = 0.60       # 支撑相占比
TROT_BASE_STEP   = 0.0        # 基础步长 (m)，摇杆中位时原地踏步（脚不前后移）
TROT_MAX_STEP    = 0.15       # 最大步长 (m)
TROT_STEP_HEIGHT = 0.04       # 抬腿高度 (m)
TROT_MAX_LATERAL = 0.06       # 最大横移偏移 (m)
TROT_MAX_YAW     = 0.04       # 最大偏航步长差 (m)

# Trot 相位偏移：LF+RB 同相(0)，RF+LB 错半周期(0.5)
TROT_PHASE_OFFSET = {0: 0.0, 1: 0.5, 2: 0.5, 3: 0.0}

# Trot 起始时间偏移：让四条腿初始时刻都处于轨迹中心（与站立原点一致）
TROT_START_T = TROT_DUTY_FACTOR * TROT_CYCLE_TIME / 2.0  # 0.24s

# 起立/趴下过渡时间
STAND_UP_TIME = 2.0            # 趴下→站立 (s)
LIE_DOWN_TIME = 2.5            # 站立→趴下 (s)

# 控制周期
CONTROL_DT = 0.005             # 200 Hz


# ═══════════════════════════════════════════════════════
# 机器人状态枚举
# ═══════════════════════════════════════════════════════
class RobotState(enum.Enum):
    DISABLED = 0   # 失能：电机锁定
    CROUCH   = 1   # 趴下：足端在趴下位置
    STANDING = 2   # 站立：足端在可调站立位置
    TROTTING = 3   # Trot：对角小跑


# ═══════════════════════════════════════════════════════
# 运动学辅助
# ═══════════════════════════════════════════════════════

def make_kinematics(leg):
    """构建某条腿的运动学模型。"""
    x_sign = 1.0 if leg in (0, 1) else -1.0   # 前腿+、后腿-
    y_sign = 1.0 if leg in (0, 2) else -1.0   # 左腿+、右腿-
    dx = x_sign * HIP_DX
    dy = y_sign * HIP_DY
    return vmc.LegKinematics(L_THIGH, L_CALF, dx, dy), dx, dy


# ═══════════════════════════════════════════════════════
# 足端轨迹生成（复合摆线，与 test_quad_trot 一致）
# ═══════════════════════════════════════════════════════

def cycloid_pos(s):
    """s: 0→1, 输出 0→1，首尾速度为零。"""
    if s <= 0: return 0.0
    if s >= 1: return 1.0
    return s - math.sin(2 * math.pi * s) / (2 * math.pi)


def lift_curve(s):
    """s: 0→1, 输出 0→1→0（半正弦抬腿）。"""
    if s <= 0 or s >= 1: return 0.0
    return 0.5 * (1.0 - math.cos(2 * math.pi * s))


# ═══════════════════════════════════════════════════════
# 四足硬件接口
# ═══════════════════════════════════════════════════════

class QuadHardware:
    """四条腿电机读写 + 运动学封装。"""

    def __init__(self, legs, sim):
        self.legs = legs
        self.sim = sim
        self.motors = {}
        self.q_sim = {}
        self.kinematics = {}   # leg → LegKinematics
        self.dx_s = {}         # leg → dx offset
        self.dy_s = {}         # leg → dy offset

        for leg in legs:
            kin, dx, dy = make_kinematics(leg)
            self.kinematics[leg] = kin
            self.dx_s[leg] = dx
            self.dy_s[leg] = dy

        if not sim and HAS_MOTORS:
            self._init_real()

    def _init_real(self):
        for leg in self.legs:
            entry = MOTOR_MAP[leg]
            ids = entry[:3]
            can_if = entry[3]
            for j, mid in enumerate(ids):
                m = motors_py.MotorDriver.create_motor(mid, "CAN", can_if, "LRO_CAN", 2)
                m.init_motor()
                self.motors[(leg, j)] = m

    def read(self, leg):
        """读取运动学坐标系下的关节角 [abd, hip, knee]。"""
        if self.sim:
            return [self.q_sim.get((leg, j), 0.0) for j in range(3)]
        q = [0.0, 0.0, 0.0]
        for j in range(3):
            m = self.motors.get((leg, j))
            if m:
                m.refresh_motor_status()
                raw = m.get_motor_pos()
                q[j] = JOINT_SIGN[leg][j] * (raw - ZERO_OFFSET[leg][j])
        return q

    def send_mit(self, leg, q_kin, vel, kp, kd, ff):
        """发送 MIT 指令。q_kin 为运动学坐标系下的目标关节角。"""
        if self.sim:
            for j in range(3):
                key = (leg, j)
                if key not in self.q_sim:
                    self.q_sim[key] = q_kin[j]
                self.q_sim[key] += 0.5 * (q_kin[j] - self.q_sim[key])
            return
        for j in range(3):
            m = self.motors.get((leg, j))
            if m:
                raw = ZERO_OFFSET[leg][j] + JOINT_SIGN[leg][j] * q_kin[j]
                m.motor_mit_cmd(raw, vel, kp, kd, ff)

    def lock_all(self):
        if not self.sim:
            for m in self.motors.values():
                m.lock_motor()

    def unlock_all(self):
        if not self.sim:
            for m in self.motors.values():
                m.unlock_motor()

    def deinit_all(self):
        if not self.sim:
            for m in self.motors.values():
                m.lock_motor()
                m.deinit_motor()




# ═══════════════════════════════════════════════════════
# 四足遥控节点
# ═══════════════════════════════════════════════════════

class QuadrupedJoyNode(Node):
    """四足机器人遥控控制 ROS2 节点（IK + MIT 位控版）。"""

    def __init__(self, sim_mode=False):
        super().__init__("quadruped_joy_control")

        # ─── 参数 ───
        self.declare_parameter("max_vx", 0.3)
        self.declare_parameter("max_vy", 0.15)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.declare_parameter("dead_zone", 0.08)
        self.declare_parameter("cmd_timeout", 0.3)
        self.declare_parameter("stand_up_time", STAND_UP_TIME)
        self.declare_parameter("lie_down_time", LIE_DOWN_TIME)

        self.max_vx      = self.get_parameter("max_vx").value
        self.max_vy      = self.get_parameter("max_vy").value
        self.max_yaw_rate = self.get_parameter("max_yaw_rate").value
        self.dead_zone    = self.get_parameter("dead_zone").value
        self.cmd_timeout  = self.get_parameter("cmd_timeout").value
        self.stand_up_time = self.get_parameter("stand_up_time").value
        self.lie_down_time = self.get_parameter("lie_down_time").value

        self.sim_mode = sim_mode
        self.legs = [0, 1, 2, 3]

        # ─── 硬件 ───
        self.hw = QuadHardware(self.legs, sim_mode)

        # ─── IK 目标足端位置（髋坐标系，Z 为负=向下） ───
        self.foot_stand_z = DEFAULT_STAND_FOOT_Z   # 当前站立足端 Z
        self.target_foot_z = CROUCH_FOOT_Z          # 目标足端 Z（平滑追踪）
        self.saved_stand_foot_z = DEFAULT_STAND_FOOT_Z

        # ─── 状态 ───
        self.state = RobotState.DISABLED
        self.running = True
        self.last_joy_time = time.time()
        self.last_btn_state = {}
        self.last_dpad_y = 0.0
        self.step_height = TROT_STEP_HEIGHT

        # ─── Trot 步态变量 ───
        self.trot_t = 0.0
        self.trot_step_len = TROT_BASE_STEP
        self.trot_lateral = 0.0
        self.trot_yaw = 0.0
        self.pending_stand = False   # 等待足端回原点后切换站立

        # ─── 过渡控制 ───
        self.transition_active = False
        self.transition_lock = threading.Lock()

        # ─── ROS2 ───
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10)
        self.control_timer = self.create_timer(CONTROL_DT, self.control_loop)

        self._log_startup()

    def _log_startup(self):
        self.get_logger().info("四足遥控节点已就绪（IK + MIT 位控）")
        self.get_logger().info(f"  模式: {'仿真' if self.sim_mode else '实机'}")
        self.get_logger().info(f"  状态: 失能（按 RB 使能）")
        self.get_logger().info(f"  站立 Z={DEFAULT_STAND_FOOT_Z}m  趴下 Z={CROUCH_FOOT_Z}m  最高 Z={MAX_STAND_FOOT_Z}m")
        self.get_logger().info("  RB=使能/急停 | B=站立/趴下 | A=站立/Trot | RT/LT=调高度")

    # ─── 手柄回调 ───
    def joy_callback(self, msg: Joy):
        self.last_joy_time = time.time()
        buttons = msg.buttons

        # ─── RB: 使能/失能 ───
        if self._btn_rising(BTN_RB, buttons):
            if self.state == RobotState.DISABLED:
                self._enable()
            else:
                self._disable()

        if self.state == RobotState.DISABLED:
            self.last_btn_state = {i: buttons[i] for i in range(len(buttons))}
            return

        # ─── B: 站立 ↔ 趴下（STAND 步态下生效） ───
        if self._btn_rising(BTN_B, buttons):
            if self.state == RobotState.CROUCH:
                self._request_transition(RobotState.STANDING)
            elif self.state == RobotState.STANDING:
                self._request_transition(RobotState.CROUCH)

        # ─── A: 站立 ↔ Trot ───
        if self._btn_rising(BTN_A, buttons):
            if self.state in (RobotState.CROUCH, RobotState.STANDING):
                self._request_transition(RobotState.TROTTING)
            elif self.state == RobotState.TROTTING:
                # 只有摇杆回中（原地踏步）时 A 才生效
                if self._is_trotting_in_place():
                    self.pending_stand = True
                    self.get_logger().info("A: 等待足端回原点后切换站立...")
                else:
                    self.get_logger().warn("A: 请先松开摇杆（原地踏步）再按 A")

        # ─── 摇杆 → Trot 运动参数 ───
        if self.state == RobotState.TROTTING:
            lx = self._apply_dead_zone(self._get_axis(msg, AXIS_LX))
            ly = self._apply_dead_zone(self._get_axis(msg, AXIS_LY))
            rx = self._apply_dead_zone(self._get_axis(msg, AXIS_RX))

            # 左摇杆上下 → 步长（前推=前进=正步长）
            self.trot_step_len = TROT_BASE_STEP + (TROT_MAX_STEP - TROT_BASE_STEP) * abs(ly)
            if ly < 0:
                self.trot_step_len = -self.trot_step_len  # 后退

            # 左摇杆左右 → 横移偏移
            self.trot_lateral = lx * TROT_MAX_LATERAL

            # 右摇杆左右 → 偏航（左右腿步长不对称）
            self.trot_yaw = rx * TROT_MAX_YAW

            # 等待切换站立期间摇杆离开中位 → 取消等待
            if self.pending_stand and not self._is_trotting_in_place():
                self.pending_stand = False
                self.get_logger().info("摇杆离开中位，取消站立切换")
        else:
            self.trot_step_len = TROT_BASE_STEP
            self.trot_lateral = 0.0
            self.trot_yaw = 0.0

        # ─── RT/LT: 调整站立高度（仅站立姿态） ───
        # RT → 升高机身 → 足端 Z 更负（向下伸）
        # LT → 降低机身 → 足端 Z 更正（向上收）
        if self.state == RobotState.STANDING:
            rt = max(0.0, self._get_axis(msg, AXIS_RT))
            lt = max(0.0, self._get_axis(msg, AXIS_LT))
            delta = (lt - rt) * 0.0008   # RT 使足端 Z↓(高度↑)，每帧步进
            if abs(delta) > 1e-6:
                self.target_foot_z += delta
                self.target_foot_z = max(MAX_STAND_FOOT_Z,
                                         min(CROUCH_FOOT_Z, self.target_foot_z))

        # ─── 十字键上下：调整抬腿高度（仅站立姿态） ───
        if self.state in (RobotState.CROUCH, RobotState.STANDING):
            dpad_y = self._get_axis(msg, AXIS_DPAD_Y)
            if dpad_y > 0.5 and self.last_dpad_y <= 0.5:
                self.step_height = min(0.10, self.step_height + 0.01)
                self.get_logger().info(f"抬腿高度 ↑ {self.step_height:.2f}m")
            elif dpad_y < -0.5 and self.last_dpad_y >= -0.5:
                self.step_height = max(0.01, self.step_height - 0.01)
                self.get_logger().info(f"抬腿高度 ↓ {self.step_height:.2f}m")
            self.last_dpad_y = dpad_y

        self.last_btn_state = {i: buttons[i] for i in range(len(buttons))}

    # ─── 状态过渡请求 ───
    def _request_transition(self, target_state):
        """在控制线程中执行状态过渡（避免回调中阻塞）。"""
        if self.transition_active:
            self.get_logger().warn("上一个过渡仍在进行，忽略")
            return
        self.transition_active = True
        threading.Thread(target=self._do_transition, args=(target_state,),
                         daemon=True).start()

    def _do_transition(self, target_state):
        """执行 S 曲线过渡到目标状态。"""
        with self.transition_lock:
            old_state = self.state

            if target_state == RobotState.CROUCH:
                self.get_logger().info("→ 趴下姿态")
                self.target_foot_z = CROUCH_FOOT_Z
                self.foot_stand_z = CROUCH_FOOT_Z
                self._s_curve_to_foot_z(self.lie_down_time)
                self.state = RobotState.CROUCH

            elif target_state == RobotState.STANDING:
                if old_state == RobotState.TROTTING:
                    self.get_logger().info("→ 站立姿态（Trot→站立）")
                else:
                    self.get_logger().info("→ 站立姿态")
                    self.target_foot_z = self.saved_stand_foot_z
                    self.foot_stand_z = self.saved_stand_foot_z
                self._s_curve_to_foot_z(self.stand_up_time)
                self.state = RobotState.STANDING

            elif target_state == RobotState.TROTTING:
                self.get_logger().info("→ Trot 步态")
                if old_state == RobotState.STANDING:
                    self.saved_stand_foot_z = self.foot_stand_z
                # 确保站立后再进入 Trot
                if self.foot_stand_z != self.target_foot_z:
                    self.target_foot_z = self.foot_stand_z
                self._s_curve_to_foot_z(0.5)  # 快速到位
                self.trot_t = TROT_START_T  # 四腿初始都在轨迹中心，与站立原点无缝衔接
                self.trot_step_len = TROT_BASE_STEP
                self.trot_lateral = 0.0
                self.trot_yaw = 0.0
                self.state = RobotState.TROTTING

            self.transition_active = False

    def _s_curve_to_foot_z(self, duration):
        """S 曲线过渡到 target_foot_z 高度。
        读取每条腿当前关节角作为起点，关节空间插值到目标，
        避免 FK→IK 往返带来的模型偏差跳变。
        """
        # 读取四条腿当前关节角作为起点
        q_starts = {}
        for leg in self.legs:
            q_starts[leg] = self.hw.read(leg)

        # 计算目标足端位置对应的关节角
        q_targets = {}
        for leg in self.legs:
            q_targets[leg] = self._ik_foot(leg, self._foot_center_x(leg),
                                            self.hw.dy_s[leg], self.target_foot_z)

        dt = CONTROL_DT
        steps = int(duration / dt)

        for i in range(steps + 1):
            if self.state == RobotState.DISABLED:
                return
            alpha = 0.5 - 0.5 * math.cos(math.pi * (i + 1) / (steps + 1))
            self.foot_stand_z = self.target_foot_z  # 末端 Z 已到目标

            for leg in self.legs:
                q = [q_starts[leg][j] + alpha * (q_targets[leg][j] - q_starts[leg][j])
                     for j in range(3)]
                kp = KP_TRANS
                kd = KD_TRANS
                if self.state == RobotState.TROTTING:
                    kp, kd = KP_STAND, KD_STAND
                self.hw.send_mit(leg, q, 0.0, kp, kd, 0.0)
            time.sleep(dt)

    def _is_trotting_in_place(self):
        """摇杆回中判断：步长为基准值、无横移、无偏航。"""
        return (abs(abs(self.trot_step_len) - TROT_BASE_STEP) < 0.005 and
                abs(self.trot_lateral) < 0.005 and
                abs(self.trot_yaw) < 0.005)

    def _all_in_stance(self):
        """四条腿是否同时处于支撑相。
        duty=0.6 时每周期出现两次：trot_t ∈ [0, 0.08] 和 [0.4, 0.48]。
        """
        duty = TROT_DUTY_FACTOR
        for leg in self.legs:
            phase = (self.trot_t / TROT_CYCLE_TIME + TROT_PHASE_OFFSET[leg]) % 1.0
            if phase >= duty:
                return False
        return True

    def _foot_center_x(self, leg):
        """站立足端 X 原点 = 关节偏置 + 可调偏移。"""
        return self.hw.dx_s[leg] + FOOT_X_OFFSET[leg]

    def _ik_foot(self, leg, dx, dy, z):
        """计算足端在 (dx, dy, z) 处的关节角。"""
        q_vec = self.hw.kinematics[leg].inverse(vmc.Vec3(dx, dy, z))
        return [q_vec[0], q_vec[1], q_vec[2]]

    # ─── 使能 / 失能 ───
    def _enable(self):
        if self.state != RobotState.DISABLED:
            return
        self.get_logger().info("RB: 使能！S 曲线过渡到趴下姿态...")
        # 注意：init_motor() 已完成 unlock→set MIT→lock 序列，
        # 电机已处于 ENABLED + MIT 模式，无需再次操作。
        self.target_foot_z = CROUCH_FOOT_Z
        self.foot_stand_z = CROUCH_FOOT_Z
        self.transition_active = True
        threading.Thread(target=self._do_enable_transition, daemon=True).start()

    def _do_enable_transition(self):
        self.state = RobotState.CROUCH  # 先设状态，S 曲线不检查 DISABLED
        self._s_curve_to_foot_z(self.lie_down_time)
        self.transition_active = False
        self.get_logger().info("  趴下姿态就绪")

    def _disable(self):
        if self.state == RobotState.DISABLED:
            return
        self.get_logger().info("RB: 失能（急停）！锁定电机")
        self.state = RobotState.DISABLED
        self.target_foot_z = CROUCH_FOOT_Z
        self.foot_stand_z = CROUCH_FOOT_Z
        self.trot_step_len = TROT_BASE_STEP
        self.trot_lateral = 0.0
        self.trot_yaw = 0.0
        self.pending_stand = False
        self.hw.unlock_all()  # CMD_DISABLE → 电机卸力

    # ─── 控制循环 (200 Hz) ───
    def control_loop(self):
        if self.state == RobotState.DISABLED:
            return
        if self.transition_active:
            return  # 过渡由 _do_transition 线程处理

        dt = CONTROL_DT
        now = time.time()

        # 通信超时保护
        if now - self.last_joy_time > self.cmd_timeout:
            if self.state == RobotState.TROTTING:
                self.trot_step_len = TROT_BASE_STEP
                self.trot_lateral = 0.0
                self.trot_yaw = 0.0

        # 平滑追踪 target_foot_z（用于 RT/LT 调节）
        if self.state in (RobotState.CROUCH, RobotState.STANDING):
            if abs(self.foot_stand_z - self.target_foot_z) > 1e-4:
                step = HEIGHT_SMOOTH_SPEED * dt
                if self.target_foot_z > self.foot_stand_z:
                    self.foot_stand_z = min(self.foot_stand_z + step, self.target_foot_z)
                else:
                    self.foot_stand_z = max(self.foot_stand_z - step, self.target_foot_z)

        if self.state in (RobotState.CROUCH, RobotState.STANDING):
            # 静态站立：保持足端在目标位置
            for leg in self.legs:
                q = self._ik_foot(leg, self._foot_center_x(leg),
                                  self.hw.dy_s[leg], self.foot_stand_z)
                self.hw.send_mit(leg, q, 0.0, KP_STAND, KD_STAND, 0.0)

        elif self.state == RobotState.TROTTING:
            if self.pending_stand:
                # 等待四条腿同时着地后切换站立
                if self._all_in_stance():
                    self.pending_stand = False
                    self.get_logger().info("四腿同时着地，切换站立")
                    self._request_transition(RobotState.STANDING)
                else:
                    self._trot_step(dt)
            else:
                self._trot_step(dt)

    # ─── Trot 步态 ───
    def _get_yaw_tangent(self, leg):
        """返回该腿站立足端绕 COM 旋转的切线方向（单位向量）。
        切线 ⊥ (足端_COM → COM)，yaw>0 对应 CW（右转）。
        """
        # 站立足端在 COM 系下的坐标
        fx = HIP_COM_X[leg] + self._foot_center_x(leg)
        fy = HIP_COM_Y[leg] + self.hw.dy_s[leg]
        # CW 切线 = (-fy, fx)
        tx, ty = -fy, fx
        n = math.sqrt(tx * tx + ty * ty)
        if n < 1e-9:
            return 0.0, 0.0
        return tx / n, ty / n

    @staticmethod
    def _cycloid_disp(step, phase_s, in_stance):
        """沿某轴的复合摆线位移。支撑相线性后蹬，摆动相摆线前摆。"""
        if in_stance:
            return step * 0.5 - step * phase_s
        else:
            return -step * 0.5 + step * cycloid_pos(phase_s)

    # ─── Trot 步态 ───
    def _trot_step(self, dt):
        """单步 Trot 控制。
        X/Y 方向独立摆线；偏航沿足端→COM 切线方向独立摆线，
        三条摆线叠加，IK 自动分配 HIP 和 ABD 出力比例。
        """
        duty = TROT_DUTY_FACTOR
        step_x = self.trot_step_len        # X 步长（前进步）
        step_y = self.trot_lateral         # Y 步长（横移步）
        step_h = self.step_height
        yaw = self.trot_yaw                # 偏航步长（沿切线方向）

        for leg in self.legs:
            raw_phase = (self.trot_t / TROT_CYCLE_TIME + TROT_PHASE_OFFSET[leg]) % 1.0
            in_stance = raw_phase < duty
            phase_s = (raw_phase / duty) if in_stance else \
                      ((raw_phase - duty) / (1.0 - duty))

            # ── 三元摆线叠加 ──
            x_off = self._cycloid_disp(step_x, phase_s, in_stance)
            y_off = self._cycloid_disp(step_y, phase_s, in_stance)

            if abs(yaw) > 1e-6:
                tan_x, tan_y = self._get_yaw_tangent(leg)
                yaw_disp = self._cycloid_disp(yaw, phase_s, in_stance)
                x_off += tan_x * yaw_disp
                y_off += tan_y * yaw_disp

            # ── Z 方向 ──
            z = self.foot_stand_z if in_stance else \
                self.foot_stand_z + step_h * lift_curve(phase_s)

            foot = vmc.Vec3(self._foot_center_x(leg) + x_off,
                            self.hw.dy_s[leg] + y_off,
                            z)

            q = self.hw.kinematics[leg].inverse(foot)
            q_list = [q[0], q[1], q[2]]
            kp = KP_STAND if in_stance else KP_SWING
            kd = KD_STAND if in_stance else KD_SWING
            self.hw.send_mit(leg, q_list, 0.0, kp, kd, 0.0)

        self.trot_t += dt

    # ─── 辅助 ───
    @staticmethod
    def _get_axis(msg, idx):
        return float(msg.axes[idx]) if len(msg.axes) > idx else 0.0

    def _apply_dead_zone(self, val):
        return 0.0 if abs(val) < self.dead_zone else val

    def _btn_rising(self, idx, buttons):
        if len(buttons) <= idx:
            return False
        prev = self.last_btn_state.get(idx, 0)
        return buttons[idx] == 1 and prev == 0

    def shutdown(self):
        self.get_logger().info("正在关闭...")
        self.running = False
        self._disable()
        self.hw.deinit_all()
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description="四足机器人遥控控制（IK + MIT 位控）")
    parser.add_argument("--sim", action="store_true", help="仿真模式")
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = QuadrupedJoyNode(sim_mode=args.sim)

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
