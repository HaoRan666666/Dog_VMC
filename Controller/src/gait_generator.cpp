#include "vmc_controller/gait_generator.h"
#include <cmath>

namespace vmc {

void GaitGenerator::setConfig(const Config& cfg) {
    config_ = cfg;
}

void GaitGenerator::setPhaseOffsets(float lf, float rf, float lh, float rh) {
    phase_offsets_[LF] = lf;
    phase_offsets_[RF] = rf;
    phase_offsets_[LB] = lh;
    phase_offsets_[RB] = rh;
}

void GaitGenerator::setGaitType(GaitType type) {
    config_.gait_type = type;
    cycle_time_ = 0.f;

    switch (type) {
    case GaitType::TROT:
        // 对角小跑：LF+RB 同相，RF+LB 同相，相位差 180°
        config_.duty_factor = 0.50f;
        setPhaseOffsets(0.f, 0.5f, 0.5f, 0.f);
        break;
    case GaitType::WALK:
        // 爬行步态：单腿依次迈步，各差 90°
        config_.duty_factor = 0.75f;
        setPhaseOffsets(0.f, 0.25f, 0.5f, 0.75f);
        break;
    case GaitType::BOUND:
        // 跳跃步态：前腿对 + 后腿对
        config_.duty_factor = 0.50f;
        setPhaseOffsets(0.f, 0.f, 0.5f, 0.5f);
        break;
    case GaitType::PACE:
        // 同侧溜蹄：左侧同相 + 右侧同相
        config_.duty_factor = 0.50f;
        setPhaseOffsets(0.f, 0.5f, 0.f, 0.5f);
        break;
    case GaitType::STAND:
        // 四足站立：全部支撑
        config_.duty_factor = 1.0f;
        setPhaseOffsets(0.f, 0.f, 0.f, 0.f);
        break;
    }
}

GaitState GaitGenerator::update(float dt, const Vec3& body_vel) {
    (void)body_vel;  // 步频固定，速度由步长控制

    cycle_time_ += dt;
    if (cycle_time_ >= config_.cycle_duration) {
        cycle_time_ -= config_.cycle_duration;
        state_.step_count++;
    }

    state_.cycle_progress = cycle_time_ / config_.cycle_duration;

    for (int i = 0; i < LEG_COUNT; ++i) {
        float raw_phase = state_.cycle_progress + phase_offsets_[i];
        raw_phase -= std::floor(raw_phase);  // 归一化到 [0, 1)

        if (raw_phase < config_.duty_factor) {
            state_.leg_phase[i] = LegPhase::STANCE;
            state_.phase[i] = raw_phase / config_.duty_factor;  // 支撑相进度归一化
        } else {
            state_.leg_phase[i] = LegPhase::SWING;
            state_.phase[i] = (raw_phase - config_.duty_factor) / (1.f - config_.duty_factor);
        }
    }
    return state_;
}

float GaitGenerator::getSwingProgress(LegIndex leg) const {
    if (state_.leg_phase[leg] != LegPhase::SWING) return -1.f;
    return state_.phase[leg];
}

float GaitGenerator::getStanceProgress(LegIndex leg) const {
    if (state_.leg_phase[leg] != LegPhase::STANCE) return -1.f;
    return state_.phase[leg];
}

Vec3 GaitGenerator::nominalFootPosition(LegIndex leg, const RobotParams& p) {
    Vec3 offset = hipOffset(leg, p);
    // 默认：足端位于髋关节正下方，标称站立高度
    return {offset.x, offset.y, offset.z - p.nominal_stand_height};
}

Vec3 GaitGenerator::hipOffset(LegIndex leg, const RobotParams& p) {
    float x_sign = (leg == LF || leg == RF) ? 1.f : -1.f;   // 前腿 +x，后腿 -x
    float y_sign = (leg == LF || leg == LB) ? 1.f : -1.f;   // 左腿 +y，右腿 -y

    return {
        x_sign * (p.hip_offset_x_front > 0 ? p.hip_offset_x_front : -p.hip_offset_x_rear),
        y_sign * p.hip_offset_y,
        p.hip_offset_z
    };
}

} // namespace vmc
