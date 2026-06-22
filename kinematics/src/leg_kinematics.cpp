#include "kinematics/leg_kinematics.h"
#include <cmath>
#include <algorithm>
//腿部解算内容，包含正运动学、逆运动学、雅可比矩阵计算，以及足端力转换为关节力矩的函数实现
namespace vmc {

//正运动学：给定关节角度，计算足端位置
Vec3 LegKinematics::forward(const std::array<float, JOINT_COUNT>& q) const {
    float a = q[ABD], h = q[HIP], k = q[KNEE];
    float sa = std::sin(a), ca = std::cos(a);
    float sh = std::sin(h), ch = std::cos(h);
    float shk = std::sin(h + k), chk = std::cos(h + k);
    float dx = params_.dx, dy = params_.dy;

    // 大小腿在矢状面内的位置（髋俯仰轴为原点，RHR绕+y）
    float x_leg = -(params_.L1 * sh + params_.L2 * shk);         // 正h → x为负(后方)
    float R    = params_.L1 * ch + params_.L2 * chk;             // YZ 平面投影长度

    // 侧摆旋转 R_x(a) 后叠加偏置 dx:
    //   fx = dx + x_leg       （dx沿x，不受侧摆影响）
    //   fy = dy·cos(a) + R·sin(a)
    //   fz = dy·sin(a) - R·cos(a)
    Vec3 foot;
    foot.x = dx + x_leg;
    foot.y = dy * ca + R * sa;
    foot.z = dy * sa - R * ca;
    return foot;
}

//逆运动学：给定足端位置，计算关节角度
std::array<float, JOINT_COUNT> LegKinematics::inverse(
    const Vec3& foot_pos, const std::array<float, JOINT_COUNT>& q_prev) const {

    float fx = foot_pos.x, fy = foot_pos.y, fz = foot_pos.z;
    float dx = params_.dx, dy = params_.dy;
    float L1 = params_.L1, L2 = params_.L2;

    std::array<float, JOINT_COUNT> q = {0, 0, 0};

    // 第 1 步：消除 X 偏置 → 大小腿在矢状面内的 x 分量
    float x_leg = fx - dx;

    // 第 2 步：求 YZ 平面投影长度 R
    //   fy² + fz² = dy² + R²  →  R = √(fy² + fz² - dy²)
    float sq = fy * fy + fz * fz - dy * dy;
    float R = (sq > 0.f) ? std::sqrt(sq) : 0.f;

    // 第 3 步：求侧摆角 a
    //   解线性方程组 [dy   R ] [cos(a)] = [fy]
    //                [-R  dy] [sin(a)]   [fz]
    float denom = dy * dy + R * R;
    float cos_a, sin_a;
    if (denom < 1e-12f) {
        cos_a = 1.f; sin_a = 0.f;
    } else {
        cos_a = (dy * fy - R * fz) / denom;
        sin_a = (R * fy + dy * fz) / denom;
    }
    q[ABD] = std::atan2(sin_a, cos_a);

    // 第 4 步：二连杆 IK（矢状面内，坐标 (x_leg, R)）
    float r = std::sqrt(x_leg * x_leg + R * R);

    float r_min = std::abs(L1 - L2);
    float r_max = L1 + L2;
    if (r > r_max) r = r_max;
    if (r < r_min) r = r_min;

    float cos_knee_interior = (L1 * L1 + L2 * L2 - r * r) / (2.f * L1 * L2);
    cos_knee_interior = std::clamp(cos_knee_interior, -1.f, 1.f);
    // 第 5 步：关节角（膝前解：大腿后倾，膝前弯，膝盖位于髋后方）
    //   beta：足端在矢状面内相对于 +R 方向的夹角
    //   alpha：大腿与髋-足连线夹角
    float beta = std::atan2(x_leg, R);
    float cos_alpha = (L1 * L1 + r * r - L2 * L2) / (2.f * L1 * r);
    cos_alpha = std::clamp(cos_alpha, -1.f, 1.f);
    float alpha = std::acos(cos_alpha);
    q[HIP] = alpha - beta;                               // 大腿后倾
    q[KNEE] = -(M_PI - std::acos(cos_knee_interior));    // 膝前弯（负角度）

    return q;
}

//雅可比矩阵：给定关节角度，计算足端位置对关节角度的偏导数
void LegKinematics::jacobian(const std::array<float, JOINT_COUNT>& q, Mat3& J) const {
    float a = q[ABD], h = q[HIP], k = q[KNEE];
    float L1 = params_.L1, L2 = params_.L2;
    float dy = params_.dy;

    float sa = std::sin(a), ca = std::cos(a);
    float ch = std::cos(h), chk = std::cos(h + k);
    float sh = std::sin(h), shk = std::sin(h + k);

    // R 及其对 h, k 的偏导
    float R = L1 * ch + L2 * chk;
    float dR_dh = -L1 * sh - L2 * shk;
    float dR_dk = -L2 * shk;

    // x_leg 偏导（dx 为常数，不影响）
    float dx_dh = -(L1 * ch + L2 * chk);
    float dx_dk = -L2 * chk;

    // fy = dy·cos(a) + R·sin(a)
    float dy_da = -dy * sa + R * ca;
    float dy_dh = dR_dh * sa;
    float dy_dk = dR_dk * sa;

    // fz = dy·sin(a) - R·cos(a)
    float dz_da = dy * ca + R * sa;
    float dz_dh = -dR_dh * ca;
    float dz_dk = -dR_dk * ca;

    // 组装雅可比 J[行=足端坐标][列=关节]
    J.m[0] = 0.f;      J.m[1] = dx_dh;    J.m[2] = dx_dk;
    J.m[3] = dy_da;    J.m[4] = dy_dh;    J.m[5] = dy_dk;
    J.m[6] = dz_da;    J.m[7] = dz_dh;    J.m[8] = dz_dk;
}

std::array<float, JOINT_COUNT> LegKinematics::footForceToTorques(
    const std::array<float, JOINT_COUNT>& q, const Vec3& foot_force) const {

    Mat3 J;
    jacobian(q, J);
    Mat3 JT = J.transpose();

    // τ = Jᵀ · F
    std::array<float, JOINT_COUNT> tau;
    tau[0] = JT.m[0] * foot_force.x + JT.m[1] * foot_force.y + JT.m[2] * foot_force.z;
    tau[1] = JT.m[3] * foot_force.x + JT.m[4] * foot_force.y + JT.m[5] * foot_force.z;
    tau[2] = JT.m[6] * foot_force.x + JT.m[7] * foot_force.y + JT.m[8] * foot_force.z;
    return tau;
}

} // namespace vmc
