#pragma once

#include "vmc_controller/types.h"
#include <vector>

namespace vmc {

/**
 * 复合摆线足端轨迹规划器。
 *
 * 摆动相：足端抬起并向前移动，采用复合摆线曲线平滑过渡。
 *   x(s) = x_0 + L * (s - sin(2πs) / 2π)
 *   z(s) = z_0 + H * (1 - cos(2πs)) / 2
 * 其中 s = t / T_swing 为归一化相位 [0, 1]。
 *
 * 支撑相：足端着地，相对于髋关节向后匀速移动，推动机身前进。
 */
class TrajectoryPlanner {
public:
    /// 摆动相参数
    struct SwingParams {
        float step_length = 0.06f;    // 前进步长 (m)
        float step_height = 0.04f;    // 抬腿高度 (m)
        float step_width_offset = 0.0f; // 摆动时的横向偏移
        float swing_duration = 0.15f; // 摆动持续时间 (s)
    };

    /// 支撑相参数
    struct StanceParams {
        float stance_duration = 0.15f; // 支撑持续时间 (s)
        float nominal_height = 0.25f;  // 髋关节以下标称站立高度 (m)，正方向向下
    };

    TrajectoryPlanner() = default;

    void setSwingParams(const SwingParams& p) { swing_params_ = p; }
    void setStanceParams(const StanceParams& p) { stance_params_ = p; }

    /**
     * 规划从起始位置到目标落点的摆动轨迹。
     * @param start_pos   摆动起始足端位置（髋关节坐标系）
     * @param target_step 从起始点到目标落点的位移向量（髋关节 XY 平面）
     * @param num_points  插值点数量
     * @return 足端轨迹采样点序列（髋关节坐标系）
     */
    std::vector<FootTrajectoryPoint> planSwing(
        const Vec3& start_pos, const Vec3& target_step, int num_points = 20) const;

    /**
     * 规划支撑相轨迹：足端相对髋关节向后移动。
     * @param start_pos  着地时刻足端位置（髋关节坐标系）
     * @param body_vel   期望机身速度 (m/s)，机身坐标系
     * @param num_points 插值点数量
     * @return 足端轨迹采样点序列（髋关节坐标系）
     */
    std::vector<FootTrajectoryPoint> planStance(
        const Vec3& start_pos, const Vec3& body_vel, int num_points = 20) const;

    /**
     * 计算复合摆线在归一化相位 s ∈ [0,1] 处的位置值。
     * @return 归一化位移 [0, 1]。
     */
    static float compositeCycloidPos(float s);

    /**
     * 计算复合摆线的速度导数。
     * @return 归一化速度（需乘以 1/T_swing 得到实际速度）。
     */
    static float compositeCycloidVel(float s);

    /**
     * 计算抬腿曲线在归一化相位 s ∈ [0,1] 处的位置值。
     * s=0 时返回值 0，s=0.5 时达到峰值，s=1 时返回值 0。
     */
    static float liftCurvePos(float s);

    /// 抬腿曲线速度导数
    static float liftCurveVel(float s);

private:
    SwingParams swing_params_;
    StanceParams stance_params_;
};

} // namespace vmc
