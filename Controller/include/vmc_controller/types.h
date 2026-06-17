#pragma once

#include <array>
#include <cstdint>

namespace vmc {

// ─── 三维向量 ───
struct Vec3 {
    float x = 0.f, y = 0.f, z = 0.f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    float norm() const;
    Vec3 normalized() const;
};

float dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);

// ─── 3x3 矩阵 ───
struct Mat3 {
    float m[9] = {0};

    Mat3() = default;
    static Mat3 zero();
    static Mat3 identity();
    static Mat3 rotationX(float angle);
    static Mat3 rotationY(float angle);
    static Mat3 rotationZ(float angle);

    Vec3 operator*(const Vec3& v) const;
    Mat3 operator*(const Mat3& o) const;
    Mat3 transpose() const;
};

// ─── 机器人物理参数 ───
struct RobotParams {
    // 机身尺寸（米）
    float body_length = 0.45f;    // 前后方向
    float body_width = 0.25f;     // 左右方向
    float body_height = 0.05f;    // 机身厚度

    // 腿部连杆长度（米）
    float hip_dx = 0.06f;   // 外展轴→髋俯仰轴 X向偏置 (m)，前正后负
    float hip_dy = 0.082f;  // 外展轴中心线→大小腿平面 Y向偏置 (m)，左正右负
    float thigh_length = 0.2125f;   // 大腿长度
    float calf_length = 0.25025f;   // 小腿长度

    // 髋关节在机身坐标系中的位置（x前, y左, z上）
    float hip_offset_x_front = 0.15f;
    float hip_offset_x_rear = -0.15f;
    float hip_offset_y = 0.12f;
    float hip_offset_z = 0.0f;

    // 整机质量（千克）
    float body_mass = 5.0f;

    // 默认站立高度：髋关节到地面的距离（米）
    float nominal_stand_height = 0.25f;

    // 关节角度限位（弧度）
    float abd_min = -0.6f, abd_max = 0.6f;
    float hip_min = -1.2f, hip_max = 1.2f;
    float knee_min = -2.4f, knee_max = 0.0f;  // 膝前弯：负角度

    // 关节力矩限位（牛·米）
    float abd_torque_limit = 10.f;
    float hip_torque_limit = 20.f;
    float knee_torque_limit = 30.f;

    // 关节零点偏移：q_kinematic = q_motor_raw - zero_offset
    // [腿][关节] 顺序：[LF, RF, LB, RB] × [ABD, HIP, KNEE]
    std::array<std::array<float, 3>, 4> joint_zero_offset = {{
        {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
    }};
};

// ─── 腿编号枚举 ───
enum LegIndex : uint8_t {
    LF = 0,  // 左前腿
    RF = 1,  // 右前腿
    LB = 2,  // 左后腿
    RB = 3,  // 右后腿
    LEG_COUNT = 4
};

// ─── 单腿关节编号枚举 ───
enum JointIndex : uint8_t {
    ABD = 0,   // 髋侧摆（横滚）
    HIP = 1,   // 髋俯仰
    KNEE = 2,  // 膝俯仰
    JOINT_COUNT = 3
};

// ─── 单腿关节状态 ───
struct JointState {
    std::array<float, JOINT_COUNT> position = {0, 0, 0};  // 角度 (rad)
    std::array<float, JOINT_COUNT> velocity = {0, 0, 0};  // 角速度 (rad/s)
    std::array<float, JOINT_COUNT> torque = {0, 0, 0};    // 力矩 (Nm)
};

// ─── 整机关节状态 ───
struct RobotJointState {
    std::array<JointState, LEG_COUNT> legs;
};

// ─── 足端在机身坐标系中的状态 ───
struct FootState {
    Vec3 position;   // 位置 (m)，机身坐标系
    Vec3 velocity;   // 速度 (m/s)
    bool in_contact = false;  // 是否着地
};

// ─── 机身状态估计 ───
struct BodyState {
    Vec3 position = {0, 0, 0};         // 世界坐标系位置 (m)
    Vec3 velocity = {0, 0, 0};         // 世界坐标系速度 (m/s)
    Vec3 euler = {0, 0, 0};           // 横滚、俯仰、偏航 (rad)
    Vec3 angular_velocity = {0, 0, 0}; // 机身坐标系角速度 (rad/s)
    Vec3 linear_accel = {0, 0, -9.81f}; // 机身坐标系线加速度 (m/s^2)
};

// ─── 单腿步态相位 ───
enum class LegPhase : uint8_t {
    STANCE = 0,  // 支撑相
    SWING = 1    // 摆动相
};

// ─── 步态状态 ───
struct GaitState {
    std::array<LegPhase, LEG_COUNT> leg_phase;  // 各腿当前相位
    std::array<float, LEG_COUNT> phase;          // 当前相位内的进度 [0, 1]
    float cycle_progress = 0.f;                   // 步态周期进度 [0, 1]
    uint32_t step_count = 0;                      // 累计步数
};

// ─── 足端轨迹采样点 ───
struct FootTrajectoryPoint {
    Vec3 position;            // 目标足端位置（髋关节坐标系）
    Vec3 velocity;            // 目标足端速度
    Vec3 feedforward_force;   // 前馈力（可选）
};

// ─── 单个电机的控制指令 ───
struct MotorCommand {
    float position = 0.f;          // 目标位置 (rad)
    float velocity = 0.f;          // 目标速度 (rad/s)
    float kp = 0.f;                // 位置刚度
    float kd = 0.f;                // 速度阻尼
    float feedforward_torque = 0.f; // 前馈力矩 (Nm)
};

// ─── 整机电机的控制指令 ───
struct RobotControlCommand {
    std::array<std::array<MotorCommand, JOINT_COUNT>, LEG_COUNT> legs;
};

// ─── 期望机身运动指令 ───
struct RobotMotionCommand {
    Vec3 target_velocity = {0, 0, 0};      // 目标机身速度 (m/s)，机身坐标系
    float target_yaw_rate = 0.f;           // 目标偏航角速度 (rad/s)
    float target_body_height = 0.0f;       // 目标高度偏移 (m)，0 表示标称站立高度
    Vec3 target_euler = {0, 0, 0};         // 目标机身姿态 (rad)
};

} // namespace vmc
