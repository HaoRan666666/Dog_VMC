#include "vmc_controller/state_estimator.h"
#include <cmath>

namespace vmc {

void StateEstimator::updateImu(float dt, const Vec3& accel, const Vec3& gyro) {
    if (dt <= 0.f) return;

    // 加速度计低通滤波，抑制高频噪声
    float alpha_acc = 1.f - std::exp(-config_.accel_lpf_cutoff * dt);
    accel_filtered_.x += alpha_acc * (accel.x - accel_filtered_.x);
    accel_filtered_.y += alpha_acc * (accel.y - accel_filtered_.y);
    accel_filtered_.z += alpha_acc * (accel.z - accel_filtered_.z);

    // 由加速度计方向推算重力方向 → 横滚/俯仰角
    Vec3 rp_acc = accelToRollPitch(accel_filtered_);

    // 陀螺仪直接积分
    float roll_gyro  = body_state_.euler.x + gyro.x * dt;
    float pitch_gyro = body_state_.euler.y + gyro.y * dt;

    // 互补滤波：α·陀螺积分 + (1-α)·加速度计观测
    float a = config_.alpha;
    body_state_.euler.x = a * roll_gyro  + (1.f - a) * rp_acc.x;
    body_state_.euler.y = a * pitch_gyro + (1.f - a) * rp_acc.y;
    body_state_.euler.z += gyro.z * dt;  // 偏航角：仅陀螺积分（无磁力计修正）

    body_state_.angular_velocity = gyro;
    body_state_.linear_accel = accel_filtered_;
}

void StateEstimator::reset() {
    body_state_ = BodyState{};
    accel_filtered_ = {0, 0, -9.81f};
}

Vec3 StateEstimator::accelToRollPitch(const Vec3& accel) {
    // 机身坐标系：x 前，y 左，z 上
    // 横滚：atan2(ay, sqrt(ax² + az²))，倾转时重力在 y 轴产生分量
    // 俯仰：atan2(ax, sqrt(ay² + az²))，倾转时重力在 x 轴产生分量
    float ax = accel.x, ay = accel.y, az = accel.z;
    float roll  = std::atan2(ay, std::sqrt(ax * ax + az * az));
    float pitch = std::atan2(ax, std::sqrt(ay * ay + az * az));
    return {roll, pitch, 0.f};
}

} // namespace vmc
