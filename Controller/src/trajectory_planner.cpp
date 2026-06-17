#include "vmc_controller/trajectory_planner.h"
#include <cmath>

namespace vmc {

static constexpr float TWO_PI = 2.0f * M_PI;

// 复合摆线位置曲线：s - sin(2πs) / 2π
// 起点速度为零，终点速度为零，中间平滑过渡
float TrajectoryPlanner::compositeCycloidPos(float s) {
    if (s <= 0.f) return 0.f;
    if (s >= 1.f) return 1.f;
    return s - std::sin(TWO_PI * s) / TWO_PI;
}

// 复合摆线速度导数：1 - cos(2πs)
float TrajectoryPlanner::compositeCycloidVel(float s) {
    if (s <= 0.f || s >= 1.f) return 0.f;
    return 1.f - std::cos(TWO_PI * s);
}

// 抬腿高度曲线（半正弦波）：(1 - cos(2πs)) / 2
float TrajectoryPlanner::liftCurvePos(float s) {
    if (s <= 0.f) return 0.f;
    if (s >= 1.f) return 0.f;
    return 0.5f * (1.f - std::cos(TWO_PI * s));
}

// 抬腿速度导数
float TrajectoryPlanner::liftCurveVel(float s) {
    if (s <= 0.f || s >= 1.f) return 0.f;
    return M_PI * std::sin(TWO_PI * s);
}

std::vector<FootTrajectoryPoint> TrajectoryPlanner::planSwing(
    const Vec3& start_pos, const Vec3& target_step, int num_points) const {

    std::vector<FootTrajectoryPoint> traj(num_points);
    const float inv_T = 1.f / swing_params_.swing_duration;

    for (int i = 0; i < num_points; ++i) {
        float s = static_cast<float>(i) / static_cast<float>(num_points - 1);

        // 复合摆线 → 水平面 (XY) 运动
        float cx = compositeCycloidPos(s);
        float cvx = compositeCycloidVel(s);

        // 半正弦 → 垂直面 (Z) 抬腿运动
        float cz = liftCurvePos(s);
        float cvz = liftCurveVel(s);

        FootTrajectoryPoint& pt = traj[i];
        // 髋坐标系中 Z 正向朝上，抬腿时 Z 减小（更接近零或正）
        pt.position.x = start_pos.x + target_step.x * cx;
        pt.position.y = start_pos.y + target_step.y * cx;
        pt.position.z = start_pos.z - swing_params_.step_height * cz;

        pt.velocity.x = target_step.x * cvx * inv_T;
        pt.velocity.y = target_step.y * cvx * inv_T;
        pt.velocity.z = -swing_params_.step_height * cvz * inv_T;
    }
    return traj;
}

std::vector<FootTrajectoryPoint> TrajectoryPlanner::planStance(
    const Vec3& start_pos, const Vec3& body_vel, int num_points) const {

    std::vector<FootTrajectoryPoint> traj(num_points);
    const float T = stance_params_.stance_duration;

    for (int i = 0; i < num_points; ++i) {
        float s = static_cast<float>(i) / static_cast<float>(num_points - 1);
        float t = s * T;

        FootTrajectoryPoint& pt = traj[i];
        // 支撑相：足端相对髋向后匀速运动，推动机身前进
        pt.position.x = start_pos.x - body_vel.x * t;
        pt.position.y = start_pos.y - body_vel.y * t;
        pt.position.z = start_pos.z;  // 保持触地高度不变

        pt.velocity.x = -body_vel.x;
        pt.velocity.y = -body_vel.y;
        pt.velocity.z = 0.f;
    }
    return traj;
}

} // namespace vmc
