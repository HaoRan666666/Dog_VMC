#pragma once

#include "vmc_controller/types.h"

namespace vmc {

/**
 * 步态类型枚举。
 */
enum class GaitType : uint8_t {
    TROT = 0,   // 对角小跑：LF+RB 同相，RF+LB 同相
    WALK = 1,   // 爬行：单腿依次迈步
    BOUND = 2,  // 跳跃：前双腿同相，后双腿同相
    PACE = 3,   // 同侧溜蹄：左侧同相，右侧同相
    STAND = 4,  // 四足站立
};

/**
 * 基于相位的步态生成器。
 *
 * 每条腿在步态周期中有一个相位偏移 [0, 1)。
 * 对于给定的步态类型，偏移量是预定义的。
 * 周期计数器按照步频推进。
 *
 * 相位 ∈ [0, duty_factor) = 支撑相，[duty_factor, 1) = 摆动相。
 */
class GaitGenerator {
public:
    struct Config {
        GaitType gait_type = GaitType::TROT;
        float cycle_duration = 0.7f;   // 完整步态周期时长 (s)
        float duty_factor = 0.60f;       // 支撑相占空比 [0, 1]
    };

    GaitGenerator() { setGaitType(GaitType::TROT); }

    void setConfig(const Config& cfg);
    const Config& config() const { return config_; }

    /**
     * 设置步态类型并自动配置相位偏移和默认占空比。
     */
    void setGaitType(GaitType type);

    /**
     * 推进步态时钟。
     * @param dt       时间步长 (s)
     * @param body_vel 当前机身速度（用于调制步频，速度越快步频越高）
     * @return 更新后的步态状态
     */
    GaitState update(float dt, const Vec3& body_vel);

    /**
     * 获取指定腿的摆动相进度 [0, 1]。
     * 0 = 摆动开始，1 = 摆动结束。
     * 若该腿处于支撑相，返回 -1。
     */
    float getSwingProgress(LegIndex leg) const;

    /**
     * 获取指定腿的支撑相进度 [0, 1]。
     * 若该腿处于摆动相，返回 -1。
     */
    float getStanceProgress(LegIndex leg) const;

    const GaitState& state() const { return state_; }

    /**
     * 获取站立状态下足端在髋坐标系中的默认位置。
     */
    static Vec3 nominalFootPosition(LegIndex leg, const RobotParams& params);

    /**
     * 获取指定腿髋关节在机身坐标系中的偏移量。
     */
    static Vec3 hipOffset(LegIndex leg, const RobotParams& params);

private:
    Config config_;
    GaitState state_;
    float cycle_time_ = 0.f;
    std::array<float, LEG_COUNT> phase_offsets_ = {0, 0.5f, 0.5f, 0};  // per-leg phase offset

    void setPhaseOffsets(float lf, float rf, float lh, float rh);
};

} // namespace vmc
