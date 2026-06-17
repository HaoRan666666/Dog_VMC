#include "vmc_controller/quadruped_controller.h"
#include <cmath>
#include <algorithm>

namespace vmc {

QuadrupedController::QuadrupedController(const RobotParams& params)
    : robot_params_(params) {
    applyConfig();
}

QuadrupedController::QuadrupedController(const RobotParams& params, const Config& config)
    : robot_params_(params), config_(config) {
    applyConfig();
}

void QuadrupedController::applyConfig() {
    // 配置四条腿的运动学模型
    LegKinematics::Params kp;
    kp.L1 = robot_params_.thigh_length;
    kp.L2 = robot_params_.calf_length;
    for (int i = 0; i < LEG_COUNT; ++i) {
        // 偏置符号：前腿 +dx，后腿 -dx；左腿 +dy，右腿 -dy
        float x_sign = (i == LF || i == RF) ? 1.f : -1.f;
        float y_sign = (i == LF || i == LB) ? 1.f : -1.f;
        kp.dx = x_sign * robot_params_.hip_dx;
        kp.dy = y_sign * robot_params_.hip_dy;
        leg_kin_[i] = LegKinematics(kp);
    }

    // 配置摆动相轨迹规划参数
    TrajectoryPlanner::SwingParams sp;
    sp.step_length = config_.step_length;
    sp.step_height = config_.step_height;
    sp.swing_duration = config_.swing_duration;
    trajectory_planner_.setSwingParams(sp);

    // 配置支撑相轨迹规划参数
    TrajectoryPlanner::StanceParams stp;
    stp.stance_duration = config_.stance_duration;
    stp.nominal_height = robot_params_.nominal_stand_height;
    trajectory_planner_.setStanceParams(stp);

    // 配置步态生成器
    GaitGenerator::Config gc;
    gc.gait_type = config_.gait_type;
    gc.cycle_duration = config_.cycle_duration;
    gc.duty_factor = config_.duty_factor;
    gait_gen_.setConfig(gc);
    gait_gen_.setGaitType(config_.gait_type);

    // 配置平衡控制器（需要各腿独立的 dx/dy 偏置）
    for (int i = 0; i < LEG_COUNT; ++i) {
        float x_sign = (i == LF || i == RF) ? 1.f : -1.f;
        float y_sign = (i == LF || i == LB) ? 1.f : -1.f;
        kp.dx = x_sign * robot_params_.hip_dx;
        kp.dy = y_sign * robot_params_.hip_dy;
        balance_ctrl_.setLegKinematics(i, kp);
    }
    BalanceController::Gains bg;
    balance_ctrl_.setGains(bg);

    // 初始化足端位置与摆动/支撑起始位置（标称站立姿态）
    for (int leg = 0; leg < LEG_COUNT; ++leg) {
        foot_positions_[leg] = GaitGenerator::nominalFootPosition(
            static_cast<LegIndex>(leg), robot_params_);
        swing_start_positions_[leg] = foot_positions_[leg];
        stance_start_positions_[leg] = foot_positions_[leg];
    }
}

void QuadrupedController::setConfig(const Config& cfg) {
    config_ = cfg;
    applyConfig();
}

void QuadrupedController::updateImu(float dt, const Vec3& accel, const Vec3& gyro) {
    estimator_.updateImu(dt, accel, gyro);
}

// ─── 零点标定 ───

void QuadrupedController::setJointFeedbackRaw(const RobotJointState& raw_state) {
    for (int leg = 0; leg < LEG_COUNT; ++leg) {
        for (int j = 0; j < JOINT_COUNT; ++j) {
            joint_state_.legs[leg].position[j] =
                raw_state.legs[leg].position[j]
                - robot_params_.joint_zero_offset[leg][j];
            joint_state_.legs[leg].velocity[j] = raw_state.legs[leg].velocity[j];
            joint_state_.legs[leg].torque[j] = raw_state.legs[leg].torque[j];
        }
    }
}

void QuadrupedController::setJointZeroOffsets(
    const std::array<std::array<float, JOINT_COUNT>, LEG_COUNT>& offsets) {
    robot_params_.joint_zero_offset = offsets;
}

float QuadrupedController::rawToKinematic(LegIndex leg, JointIndex joint,
                                           float raw_angle) const {
    return raw_angle - robot_params_.joint_zero_offset[leg][joint];
}

float QuadrupedController::kinematicToRaw(LegIndex leg, JointIndex joint,
                                           float kin_angle) const {
    return kin_angle + robot_params_.joint_zero_offset[leg][joint];
}

void QuadrupedController::calibrateJoint(LegIndex leg, JointIndex joint,
                                          float motor_reading, float kinematic_angle) {
    robot_params_.joint_zero_offset[leg][joint] = motor_reading - kinematic_angle;
}

void QuadrupedController::calibrateFromReferencePose(
    const RobotJointState& raw_motor_state,
    const std::array<Vec3, LEG_COUNT>& foot_positions) {

    for (int leg = 0; leg < LEG_COUNT; ++leg) {
        // 由实测足端位置计算 IK 期望关节角
        Vec3 foot_in_hip = foot_positions[leg]
            - GaitGenerator::hipOffset(static_cast<LegIndex>(leg), robot_params_);
        auto q_expected = leg_kin_[leg].inverse(foot_in_hip);

        // 用电机原始读数计算偏移量
        for (int j = 0; j < JOINT_COUNT; ++j) {
            robot_params_.joint_zero_offset[leg][j] =
                raw_motor_state.legs[leg].position[j] - q_expected[j];
        }
    }
}

Vec3 QuadrupedController::planSwingFootTarget(LegIndex leg, float s, float step_len,
                                               const Vec3& hip_offset, float yaw_rate) const {
    float cx = TrajectoryPlanner::compositeCycloidPos(s);
    float cz = TrajectoryPlanner::liftCurvePos(s);

    Vec3 target;
    target.x = swing_start_positions_[leg].x;
    target.y = swing_start_positions_[leg].y;

    // 前进方向步长
    target.x += step_len * cx;

    // 偏航旋转：横向步长分量，前腿和后腿方向相反
    // CCW旋转时前腿向内迈、后腿向外迈
    float step_y = -yaw_rate * hip_offset.x * config_.swing_duration;
    target.y += step_y * cx;

    // 抬腿弧线
    target.z = swing_start_positions_[leg].z - config_.step_height * cz;

    return target;
}

Vec3 QuadrupedController::planStanceFootTarget(LegIndex leg, float s,
                                                float yaw_rate, const Vec3& foot_body) const {
    // 支撑相：足端相对髋关节向后移动，叠加旋转分量
    // 足端固定于世界，机身平移 v + 旋转 ω → 髋坐标系中足端速度 = -v - ω×r
    Vec3 target = stance_start_positions_[leg];
    float dt_stance = s * config_.stance_duration;

    // 平移分量
    target.x -= motion_cmd_.target_velocity.x * dt_stance;
    target.y -= motion_cmd_.target_velocity.y * dt_stance;

    // 旋转分量：v_rot = -ω×r_body = [ωz·y_body, -ωz·x_body, 0]
    target.x += yaw_rate * foot_body.y * dt_stance;
    target.y -= yaw_rate * foot_body.x * dt_stance;

    return target;
}

RobotControlCommand QuadrupedController::step(float dt) {
    RobotControlCommand output = {};
    if (dt <= 0.f) return output;

    // ─── 1. 根据目标速度动态计算步长（步频固定，速度靠步长调节） ───
    float speed = std::sqrt(motion_cmd_.target_velocity.x * motion_cmd_.target_velocity.x
                          + motion_cmd_.target_velocity.y * motion_cmd_.target_velocity.y);
    float dyn_step_length = speed * gait_gen_.config().duty_factor * gait_gen_.config().cycle_duration;
    // 钳制在合理范围内，config_.step_length 作为上限保护
    if (dyn_step_length > config_.step_length) dyn_step_length = config_.step_length;
    if (dyn_step_length < 0.01f) dyn_step_length = 0.01f;  // 最小步长，保证有摆动

    // ─── 2. 更新步态相位 ───
    GaitState gait = gait_gen_.update(dt, motion_cmd_.target_velocity);

    // ─── 3. 检测相位切换（支撑→摆动、摆动→支撑） ───
    for (int leg = 0; leg < LEG_COUNT; ++leg) {
        LegPhase prev = prev_gait_state_.leg_phase[leg];
        LegPhase curr = gait.leg_phase[leg];

        if (prev == LegPhase::STANCE && curr == LegPhase::SWING) {
            // 离地瞬间：记录摆动起始位置
            swing_start_positions_[leg] = foot_positions_[leg];
        }
        if (prev == LegPhase::SWING && curr == LegPhase::STANCE) {
            // 着地瞬间：记录支撑起始位置
            stance_start_positions_[leg] = foot_positions_[leg];
        }
    }

    // ─── 3. 为每条腿规划足端目标 ───
    float yaw_rate = motion_cmd_.target_yaw_rate;
    std::array<Vec3, LEG_COUNT> foot_targets;

    for (int leg = 0; leg < LEG_COUNT; ++leg) {
        auto leg_idx = static_cast<LegIndex>(leg);
        Vec3 hip_off = GaitGenerator::hipOffset(leg_idx, robot_params_);

        if (gait.leg_phase[leg] == LegPhase::SWING) {
            float s = gait.phase[leg];
            foot_targets[leg] = planSwingFootTarget(leg_idx, s, dyn_step_length, hip_off, yaw_rate);
        } else {
            float s = gait.phase[leg];
            // 足端在机身坐标系中的位置 = 髋偏移 + 足端髋坐标系位置
            Vec3 foot_body = {hip_off.x + foot_positions_[leg].x,
                              hip_off.y + foot_positions_[leg].y,
                              hip_off.z + foot_positions_[leg].z};
            foot_targets[leg] = planStanceFootTarget(leg_idx, s, yaw_rate, foot_body);
        }
    }

    // ─── 4. 逆运动学计算目标关节角 ───
    std::array<std::array<float, JOINT_COUNT>, LEG_COUNT> target_joint_pos;
    std::array<Vec3, LEG_COUNT> actual_foot_pos;
    std::array<std::array<float, JOINT_COUNT>, LEG_COUNT> all_joint_pos;

    for (int leg = 0; leg < LEG_COUNT; ++leg) {
        all_joint_pos[leg] = joint_state_.legs[leg].position;
        auto q = leg_kin_[leg].inverse(foot_targets[leg], joint_state_.legs[leg].position);
        target_joint_pos[leg] = q;
        actual_foot_pos[leg] = leg_kin_[leg].forward(joint_state_.legs[leg].position);
    }

    // ─── 5. 计算 VMC 支撑腿力矩 ───
    auto vmc_torques = balance_ctrl_.computeTorques(
        estimator_.bodyState(), motion_cmd_,
        actual_foot_pos, all_joint_pos,
        gait, robot_params_);

    // ─── 6. 组装电机控制指令（MIT 模式） ───
    for (int leg = 0; leg < LEG_COUNT; ++leg) {
        bool in_swing = (gait.leg_phase[leg] == LegPhase::SWING);

        for (int j = 0; j < JOINT_COUNT; ++j) {
            MotorCommand& cmd = output.legs[leg][j];

            if (in_swing) {
                // 摆动腿：高刚度 PD 位置跟踪
                cmd.position = target_joint_pos[leg][j];
                cmd.velocity = 0.f;
                cmd.kp = config_.swing_kp;
                cmd.kd = config_.swing_kd;
                cmd.feedforward_torque = 0.f;
            } else {
                // 支撑腿：低刚度位置保持 + VMC 力矩前馈
                cmd.position = target_joint_pos[leg][j];
                cmd.velocity = 0.f;
                cmd.kp = config_.stance_kp;
                cmd.kd = config_.stance_kd;
                cmd.feedforward_torque = vmc_torques[leg][j];
            }
        }
    }

    // ─── 7. 保存状态用于下一迭代 ───
    foot_positions_ = actual_foot_pos;
    prev_gait_state_ = gait;

    return output;
}

} // namespace vmc
