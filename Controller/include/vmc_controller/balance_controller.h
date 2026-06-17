#pragma once

#include "vmc_controller/types.h"
#include "vmc_controller/leg_kinematics.h"
#include <array>

namespace vmc {

/**
 * 虚拟模型控制（VMC）平衡控制器。
 *
 * 在机身质心处计算虚拟力螺旋以跟踪期望运动，
 * 然后通过抓取矩阵的伪逆将其分配到各支撑腿足端。
 * 足端力通过雅可比转置 Jᵀ 转换为关节力矩。
 */
class BalanceController {
public:
    /// PD 控制器增益
    struct Gains {
        // 质心位置/速度增益
        float kp_com_x = 30.f, kd_com_x = 15.f;
        float kp_com_y = 30.f, kd_com_y = 15.f;
        float kp_com_z = 500.f, kd_com_z = 100.f;

        // 横滚/俯仰姿态增益
        float kp_roll = 200.f, kd_roll = 40.f;
        float kp_pitch = 200.f, kd_pitch = 40.f;

        // 偏航角速度增益
        float kd_yaw = 30.f;

        // 力分布的 Tikhonov 正则化系数
        float reg_lambda = 0.01f;

        // 摩擦锥近似约束系数
        float friction_mu = 0.7f;

        // 单足受力限幅 (N)
        float max_foot_force_z = 80.f;
        float min_foot_force_z = 5.f;    // 最小法向力（保证足端不脱离地面）
        float max_foot_force_xy = 30.f;   // 水平力限幅
    };

    BalanceController() = default;
    void setGains(const Gains& g) { gains_ = g; }
    void setLegKinematics(const LegKinematics::Params& p);
    void setLegKinematics(int leg, const LegKinematics::Params& p) { leg_kinematics_[leg] = LegKinematics(p); }

    /**
     * 为所有腿计算关节力矩。
     *
     * @param body_state     当前机身状态估计
     * @param motion_cmd     期望机身运动指令
     * @param foot_positions 当前足端在机身坐标系中的位置 [LEG_COUNT]
     * @param joint_positions 当前关节角度 [LEG_COUNT][JOINT_COUNT]
     * @param gait_state     各腿当前支撑/摆动状态
     * @param robot_params   机器人物理参数
     * @return 各腿关节力矩 [LEG_COUNT][JOINT_COUNT]
     */
    std::array<std::array<float, JOINT_COUNT>, LEG_COUNT> computeTorques(
        const BodyState& body_state,
        const RobotMotionCommand& motion_cmd,
        const std::array<Vec3, LEG_COUNT>& foot_positions,
        const std::array<std::array<float, JOINT_COUNT>, LEG_COUNT>& joint_positions,
        const GaitState& gait_state,
        const RobotParams& robot_params);

    /**
     * 计算在机身质心处施加的虚拟力螺旋 W = [F_x, F_y, F_z, τ_x, τ_y, τ_z]。
     */
    std::array<float, 6> computeVirtualWrench(
        const BodyState& body_state,
        const RobotMotionCommand& motion_cmd,
        const RobotParams& params) const;

private:
    Gains gains_;
    std::array<LegKinematics, LEG_COUNT> leg_kinematics_;

    /**
     * 构建抓取矩阵 G (6 × 3N)，其中 N 为支撑腿数量。
     * 输出 G 的平铺数组和支撑腿索引列表。
     */
    void buildGraspMatrix(
        const std::array<Vec3, LEG_COUNT>& foot_positions,
        const GaitState& gait_state,
        float G[6][12],
        int& num_stance,
        int stance_indices[4]) const;

    /**
     * 伪逆求解足端力分布：f = Gᵀ · (G·Gᵀ + λI)⁻¹ · W。
     */
    void solveFootForces(
        const float G[6][12], int num_stance,
        const int stance_indices[4],
        const std::array<float, 6>& wrench,
        Vec3 foot_forces[4]) const;

    /**
     * 将足端力钳制在摩擦锥和力限幅范围内。
     */
    void clampFootForce(Vec3& force) const;
};

} // namespace vmc
