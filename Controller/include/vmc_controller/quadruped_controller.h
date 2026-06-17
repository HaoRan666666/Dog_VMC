#pragma once

#include "vmc_controller/types.h"
#include "vmc_controller/trajectory_planner.h"
#include "vmc_controller/leg_kinematics.h"
#include "vmc_controller/gait_generator.h"
#include "vmc_controller/state_estimator.h"
#include "vmc_controller/balance_controller.h"

namespace vmc {

/**
 * 四足机器人顶层位置控制器。
 *
 * 集成子系统：复合摆线轨迹 | 步态调度 | IMU 状态估计 | VMC 平衡 | PD 跟踪
 *
 * 使用示例：
 *   QuadrupedController ctrl(params);
 *   ctrl.setMotionCommand({vx, vy, 0, yaw_rate, 0, {0,0,0}});
 *   while (running) {
 *       ctrl.updateImu(accel, gyro, dt);
 *       // 从电机读取原始角度 → 设置反馈（自动施加零点偏移）
 *       ctrl.setJointFeedbackRaw(raw_joint_state);
 *       auto commands = ctrl.step(dt);
 *       // 将 commands 通过 MIT 模式发送给电机
 *   }
 */
class QuadrupedController {
public:
    /// 控制器可调参数
    struct Config {
        float swing_kp = 80.f;         // 摆动腿位置刚度
        float swing_kd = 5.f;          // 摆动腿速度阻尼
        float stance_kp = 30.f;        // 支撑腿位置刚度
        float stance_kd = 3.f;         // 支撑腿速度阻尼
        float step_length = 0.06f;     // 步长 (m)
        float step_height = 0.04f;     // 抬腿高度 (m)
        float swing_duration = 0.15f;  // 摆动相时长 (s)
        float stance_duration = 0.15f; // 支撑相时长 (s)
        GaitType gait_type = GaitType::TROT;
        float cycle_duration = 0.70f;  // 步态周期 (s)
        float duty_factor = 0.60f;     // 支撑占空比
    };

    explicit QuadrupedController(const RobotParams& params = {});
    QuadrupedController(const RobotParams& params, const Config& config);

    void setConfig(const Config& cfg);
    const Config& config() const { return config_; }
    const RobotParams& robotParams() const { return robot_params_; }

    /// 设置期望机身运动指令
    void setMotionCommand(const RobotMotionCommand& cmd) { motion_cmd_ = cmd; }
    const RobotMotionCommand& motionCommand() const { return motion_cmd_; }

    /// 向状态估计器输入 IMU 数据
    void updateImu(float dt, const Vec3& accel, const Vec3& gyro);

    /**
     * 设置电机反馈的关节状态（已施加零点偏移的运动学角度）。
     * 如果你读取的是电机原始角度，请用 setJointFeedbackRaw()。
     */
    void setJointFeedback(const RobotJointState& state) { joint_state_ = state; }

    /**
     * 设置电机原始角度反馈，内部自动施加零点偏移转换为运动学角度。
     * q_kinematic = q_motor_raw - joint_zero_offset
     */
    void setJointFeedbackRaw(const RobotJointState& raw_state);

    /// 执行一次控制迭代
    RobotControlCommand step(float dt);

    // ─── 零点标定接口 ───

    /**
     * 单关节标定：记录电机读数与已知运动学角度的对应关系。
     * @param leg           腿编号
     * @param joint         关节编号
     * @param motor_reading 电机当前读数 (rad)
     * @param kinematic_angle 该姿态下正确的运动学角度 (rad)
     *
     * 示例（标定膝关节在机械限位处）：
     *   测量大腿-小腿内角 = 170° → 运动学膝角 = π - 170°·π/180 = 0.175 rad
     *   读取电机角度 = 2.35 rad
     *   ctrl.calibrateJoint(LB, KNEE, 2.35f, 0.175f);
     */
    void calibrateJoint(LegIndex leg, JointIndex joint,
                        float motor_reading, float kinematic_angle);

    /**
     * 通过参考姿态批量标定全部 12 个关节。
     * 将机器人置于已知姿态（如四足平放地面，测量机身高度和足端位置），
     * 用 IK 计算期望关节角，对比电机读数自动计算所有偏移量。
     *
     * @param raw_motor_state  12 个关节的电机原始读数
     * @param foot_positions   4 条腿足端在机身坐标系中的实测位置 (m)
     */
    void calibrateFromReferencePose(
        const RobotJointState& raw_motor_state,
        const std::array<Vec3, LEG_COUNT>& foot_positions);

    /**
     * 获取当前零点偏移表（可用于保存/加载）。
     */
    const std::array<std::array<float, JOINT_COUNT>, LEG_COUNT>&
        jointZeroOffsets() const { return robot_params_.joint_zero_offset; }

    /**
     * 直接设置零点偏移表（从配置文件加载）。
     */
    void setJointZeroOffsets(
        const std::array<std::array<float, JOINT_COUNT>, LEG_COUNT>& offsets);

    /**
     * 电机原始角度 → 运动学角度。
     */
    float rawToKinematic(LegIndex leg, JointIndex joint, float raw_angle) const;

    /**
     * 运动学角度 → 电机原始角度（用于发送目标位置时反算）。
     */
    float kinematicToRaw(LegIndex leg, JointIndex joint, float kin_angle) const;

    // ─── 状态访问接口 ───
    const BodyState& bodyState() const { return estimator_.bodyState(); }
    const GaitState& gaitState() const { return gait_gen_.state(); }
    const GaitGenerator& gaitGenerator() const { return gait_gen_; }
    const StateEstimator& stateEstimator() const { return estimator_; }
    const BalanceController& balanceController() const { return balance_ctrl_; }
    const LegKinematics& legKinematics(int leg) const { return leg_kin_[leg]; }
    const std::array<Vec3, LEG_COUNT>& footPositions() const { return foot_positions_; }

private:
    RobotParams robot_params_;
    Config config_;

    TrajectoryPlanner trajectory_planner_;
    std::array<LegKinematics, LEG_COUNT> leg_kin_;
    GaitGenerator gait_gen_;
    StateEstimator estimator_;
    BalanceController balance_ctrl_;

    RobotJointState joint_state_;
    RobotMotionCommand motion_cmd_;
    std::array<Vec3, LEG_COUNT> foot_positions_;
    GaitState prev_gait_state_;
    std::array<Vec3, LEG_COUNT> swing_start_positions_;
    std::array<Vec3, LEG_COUNT> stance_start_positions_;

    void applyConfig();
    Vec3 planSwingFootTarget(LegIndex leg, float swing_progress, float step_len,
                             const Vec3& hip_offset, float yaw_rate) const;
    Vec3 planStanceFootTarget(LegIndex leg, float stance_progress,
                              float yaw_rate, const Vec3& foot_body) const;
};

} // namespace vmc
