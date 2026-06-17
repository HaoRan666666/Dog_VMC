#pragma once

#include "vmc_controller/types.h"

namespace vmc {

/**
 * 三自由度腿运动学：侧摆-髋俯仰-膝俯仰（均严格遵循右手定则）。
 *
 * 髋关节坐标系：x 向前，y 向左，z 向上。
 *
 * 右手定则（RHR）关节约定：
 *   q[0] = 侧摆（绕 +x 轴），拇指指前(+x)，四指从 +y 弯向 +z
 *          → 正值使足端向 +y（左）移动
 *   q[1] = 髋俯仰（绕 +y 轴），拇指指左(+y)，四指从 +z 弯向 +x
 *          → 正值使足端向 -x（后方）移动，负值前伸
 *   q[2] = 膝俯仰（绕 +y 轴），同上 RHR
 *          → 负值使小腿向 +x 弯（膝前弯），0 为完全伸直
 *
 * 构型：膝前弯（bird-like），趴下时膝盖位于髋关节后方。
 * 零位构型：q = [0, 0, 0] → 腿完全伸直竖直下垂，
 *   足端在 (dx, dy, -(L1+L2))（dx/dy 为髋外展轴到大小腿运动平面的偏置）。
 */
class LegKinematics {
public:
    struct Params {
        float L1 = 0.20f;   // 大腿长度 (m)
        float L2 = 0.20f;   // 小腿长度 (m)
        float dx = 0.00f;   // 外展轴→髋俯仰轴 X向偏置 (m)，前腿正后腿负
        float dy = 0.00f;   // 外展轴中心线→大小腿运动平面 Y向偏置 (m)，左腿正右腿负
    };

    LegKinematics() = default;
    explicit LegKinematics(const Params& p) : params_(p) {}

    /**
     * 正向运动学：关节角度 → 足端在髋坐标系中的位置。
     * @param q 关节角度 [侧摆, 髋俯仰, 膝俯仰] (rad)
     * @return 足端位置（髋关节坐标系）
     */
    Vec3 forward(const std::array<float, JOINT_COUNT>& q) const;

    /**
     * 逆向运动学：足端在髋坐标系中的目标位置 → 关节角度。
     * 固定使用膝前解（膝盖位于髋后方，符合本机机械构型）。
     * @param foot_pos 足端目标位置（髋关节坐标系）
     * @param q_prev   上一时刻的关节角度（保留参数，当前未使用）
     * @return 关节角度 [侧摆, 髋俯仰, 膝俯仰] (rad)
     */
    std::array<float, JOINT_COUNT> inverse(const Vec3& foot_pos,
        const std::array<float, JOINT_COUNT>& q_prev = {0, 0, 0}) const;

    /**
     * 计算雅可比矩阵：关节速度 → 足端速度。
     * J[i][j] = ∂(foot_i) / ∂(q_j)。
     */
    void jacobian(const std::array<float, JOINT_COUNT>& q, Mat3& J) const;

    /**
     * 足端力 → 关节力矩：τ = Jᵀ · F_foot。
     * 用于 VMC 平衡控制中将支撑腿的地面反力转换为关节力矩。
     */
    std::array<float, JOINT_COUNT> footForceToTorques(
        const std::array<float, JOINT_COUNT>& q, const Vec3& foot_force) const;

    const Params& params() const { return params_; }

private:
    Params params_;
};

} // namespace vmc
