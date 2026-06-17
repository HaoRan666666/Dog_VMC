#include "vmc_controller/balance_controller.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace vmc {

void BalanceController::setLegKinematics(const LegKinematics::Params& p) {
    for (int i = 0; i < LEG_COUNT; ++i) {
        leg_kinematics_[i] = LegKinematics(p);
    }
}

std::array<float, 6> BalanceController::computeVirtualWrench(
    const BodyState& body_state,
    const RobotMotionCommand& cmd,
    const RobotParams& params) const {

    const Gains& g = gains_;
    std::array<float, 6> W = {0};

    float z_error = cmd.target_body_height;

    // ─── 虚拟力（质心处） ───
    // X 方向
    W[0] = g.kp_com_x * (cmd.target_velocity.x - body_state.velocity.x)
         + g.kd_com_x * (-body_state.linear_accel.x);

    // Y 方向
    W[1] = g.kp_com_y * (cmd.target_velocity.y - body_state.velocity.y)
         + g.kd_com_y * (-body_state.linear_accel.y);

    // Z 方向：重力补偿 + PD 高度控制
    W[2] = params.body_mass * 9.81f
         + g.kp_com_z * z_error
         + g.kd_com_z * (0.f - body_state.velocity.z);

    // ─── 虚拟力矩（机身坐标系） ───
    // 横滚轴
    W[3] = g.kp_roll  * (cmd.target_euler.x - body_state.euler.x)
         + g.kd_roll  * (0.f - body_state.angular_velocity.x);

    // 俯仰轴
    W[4] = g.kp_pitch * (cmd.target_euler.y - body_state.euler.y)
         + g.kd_pitch * (0.f - body_state.angular_velocity.y);

    // 偏航轴（角速度控制）
    W[5] = g.kd_yaw * (cmd.target_yaw_rate - body_state.angular_velocity.z);

    return W;
}

void BalanceController::buildGraspMatrix(
    const std::array<Vec3, LEG_COUNT>& foot_positions,
    const GaitState& gait_state,
    float G[6][12], int& num_stance,
    int stance_indices[4]) const {

    num_stance = 0;
    std::memset(G, 0, sizeof(float) * 6 * 12);

    for (int leg = 0; leg < LEG_COUNT; ++leg) {
        if (gait_state.leg_phase[leg] != LegPhase::STANCE) continue;

        int idx = num_stance;
        stance_indices[idx] = leg;
        num_stance++;

        float px = foot_positions[leg].x;
        float py = foot_positions[leg].y;
        float pz = foot_positions[leg].z;

        int col = idx * 3;  // 该腿在 G 矩阵中的起始列

        // 力的叠加：F = Σ f_i，每腿贡献单位矩阵 I₃
        G[0][col + 0] = 1.f;
        G[1][col + 1] = 1.f;
        G[2][col + 2] = 1.f;

        // 力矩的叠加：τ = Σ (p_i × f_i)
        // 叉乘矩阵 [p×] = [[0, -pz, py], [pz, 0, -px], [-py, px, 0]]
        G[3][col + 1] = -pz;    // dτ_x / df_y
        G[3][col + 2] = py;     // dτ_x / df_z
        G[4][col + 0] = pz;     // dτ_y / df_x
        G[4][col + 2] = -px;    // dτ_y / df_z
        G[5][col + 0] = -py;    // dτ_z / df_x
        G[5][col + 1] = px;     // dτ_z / df_y
    }
}

// 6×6 矩阵求逆（Gauss-Jordan 消元 + 部分选主元）
static bool invert6x6(const float A[6][6], float A_inv[6][6]) {
    float aug[6][12] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) aug[i][j] = A[i][j];
        aug[i][6 + i] = 1.f;  // 右半边是单位矩阵
    }

    for (int col = 0; col < 6; ++col) {
        // 部分选主元
        int pivot = col;
        float max_val = std::abs(aug[col][col]);
        for (int row = col + 1; row < 6; ++row) {
            if (std::abs(aug[row][col]) > max_val) {
                max_val = std::abs(aug[row][col]);
                pivot = row;
            }
        }
        if (max_val < 1e-10f) return false;  // 奇异矩阵

        if (pivot != col) {
            for (int j = 0; j < 12; ++j) std::swap(aug[col][j], aug[pivot][j]);
        }

        // 归一化主元行
        float piv_val = aug[col][col];
        for (int j = 0; j < 12; ++j) aug[col][j] /= piv_val;

        // 消去其他行
        for (int row = 0; row < 6; ++row) {
            if (row == col) continue;
            float factor = aug[row][col];
            for (int j = 0; j < 12; ++j) {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // 提取右半边作为逆矩阵
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            A_inv[i][j] = aug[i][6 + j];

    return true;
}

void BalanceController::solveFootForces(
    const float G[6][12], int num_stance,
    const int stance_indices[4],
    const std::array<float, 6>& wrench,
    Vec3 foot_forces[4]) const {

    if (num_stance == 0) return;

    int n = num_stance * 3;  // 未知量个数

    // 构造 G·Gᵀ（6×6 对称矩阵）
    float GGT[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float sum = 0.f;
            for (int k = 0; k < n; ++k) sum += G[i][k] * G[j][k];
            GGT[i][j] = sum;
        }
        GGT[i][i] += gains_.reg_lambda;  // Tikhonov 正则化
    }

    // 求逆
    float GGT_inv[6][6];
    if (!invert6x6(GGT, GGT_inv)) return;

    // f = Gᵀ · (G·Gᵀ)⁻¹ · W（Moore-Penrose 伪逆）
    float f[12] = {0};
    for (int i = 0; i < n; ++i) {
        float sum = 0.f;
        for (int j = 0; j < 6; ++j) {
            float GGTinv_W = 0.f;
            for (int k = 0; k < 6; ++k) {
                GGTinv_W += GGT_inv[j][k] * wrench[k];
            }
            sum += G[j][i] * GGTinv_W;
        }
        f[i] = sum;
    }

    // 提取各腿力并施加限幅
    for (int i = 0; i < num_stance; ++i) {
        int leg = stance_indices[i];
        foot_forces[leg] = {f[i * 3], f[i * 3 + 1], f[i * 3 + 2]};
        clampFootForce(foot_forces[leg]);
    }
}

void BalanceController::clampFootForce(Vec3& force) const {
    float fx = force.x, fy = force.y, fz = force.z;

    // 法向力限幅（保证足端稳定着地）
    if (fz < gains_.min_foot_force_z) fz = gains_.min_foot_force_z;
    if (fz > gains_.max_foot_force_z) fz = gains_.max_foot_force_z;

    // 摩擦锥约束：|f_xy| ≤ μ·fz
    float f_xy_norm = std::sqrt(fx * fx + fy * fy);
    float max_xy = gains_.friction_mu * fz;
    if (f_xy_norm > max_xy && f_xy_norm > 1e-6f) {
        float scale = max_xy / f_xy_norm;
        fx *= scale;
        fy *= scale;
    }

    // 水平力硬限幅
    if (std::abs(fx) > gains_.max_foot_force_xy) {
        fx = std::copysign(gains_.max_foot_force_xy, fx);
    }
    if (std::abs(fy) > gains_.max_foot_force_xy) {
        fy = std::copysign(gains_.max_foot_force_xy, fy);
    }

    force = {fx, fy, fz};
}

std::array<std::array<float, JOINT_COUNT>, LEG_COUNT> BalanceController::computeTorques(
    const BodyState& body_state,
    const RobotMotionCommand& motion_cmd,
    const std::array<Vec3, LEG_COUNT>& foot_positions,
    const std::array<std::array<float, JOINT_COUNT>, LEG_COUNT>& joint_positions,
    const GaitState& gait_state,
    const RobotParams& robot_params) {

    // 1. 计算虚拟力螺旋
    auto W = computeVirtualWrench(body_state, motion_cmd, robot_params);

    // 2. 构建抓取矩阵
    float G[6][12];
    int num_stance;
    int stance_indices[4];
    buildGraspMatrix(foot_positions, gait_state, G, num_stance, stance_indices);

    // 3. 将虚拟力螺旋分配到各支撑腿足端
    Vec3 foot_forces[LEG_COUNT] = {};
    solveFootForces(G, num_stance, stance_indices, W, foot_forces);

    // 4. 通过 Jᵀ 将足端力转换为关节力矩
    std::array<std::array<float, JOINT_COUNT>, LEG_COUNT> torques = {};

    for (int leg = 0; leg < LEG_COUNT; ++leg) {
        if (gait_state.leg_phase[leg] == LegPhase::STANCE) {
            auto tau = leg_kinematics_[leg].footForceToTorques(
                joint_positions[leg], foot_forces[leg]);
            torques[leg] = tau;
        }
        // 摆动腿不施加 VMC 力矩（由 PD 轨迹跟踪控制）
    }

    return torques;
}

} // namespace vmc
