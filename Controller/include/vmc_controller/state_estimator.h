#pragma once

#include "vmc_controller/types.h"

namespace vmc {

/**
 * 基于 IMU 互补滤波的姿态估计器。
 *
 * 融合陀螺仪（高频、易漂移）和加速度计（低频、噪声大）数据，
 * 估计机身的横滚角和俯仰角。
 * 偏航角仅由陀螺仪 z 轴积分得到（无磁力计则无绝对参考）。
 */
class StateEstimator {
public:
    struct Config {
        float alpha = 0.98f;           // 互补滤波增益（信任陀螺仪的比例）
        float accel_lpf_cutoff = 20.f;  // 加速度计低通滤波截止频率 (Hz)
    };

    StateEstimator() = default;
    void setConfig(const Config& cfg) { config_ = cfg; }

    /**
     * 使用 IMU 数据更新姿态估计。
     * @param dt    时间步长 (s)
     * @param accel 加速度计读数 (m/s²)，机身坐标系
     * @param gyro  陀螺仪读数 (rad/s)，机身坐标系
     */
    void updateImu(float dt, const Vec3& accel, const Vec3& gyro);

    const BodyState& bodyState() const { return body_state_; }

    /// 重置估计器（如启动时调用）
    void reset();

    /**
     * 从加速度计读数中提取横滚角和俯仰角（假设近似静止）。
     * 机身坐标系：x 前，y 左，z 上。
     */
    static Vec3 accelToRollPitch(const Vec3& accel);

private:
    Config config_;
    BodyState body_state_;
    Vec3 accel_filtered_ = {0, 0, -9.81f};  // 加速度计低通滤波值
};

} // namespace vmc
