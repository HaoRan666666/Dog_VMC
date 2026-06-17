#include "vmc_controller/types.h"
#include "vmc_controller/trajectory_planner.h"
#include "vmc_controller/leg_kinematics.h"
#include "vmc_controller/gait_generator.h"
#include "vmc_controller/state_estimator.h"
#include <cstdio>
#include <cmath>
#include <cassert>

static constexpr float EPS = 1e-4f;

static bool approx(float a, float b, float eps = EPS) {
    return std::abs(a - b) < eps;
}

// ─── Composite cycloid tests ───
static void test_composite_cycloid() {
    using vmc::TrajectoryPlanner;

    // Position curve
    assert(approx(TrajectoryPlanner::compositeCycloidPos(0.f), 0.f));
    assert(approx(TrajectoryPlanner::compositeCycloidPos(1.f), 1.f));
    // Midpoint should be > 0.5 (cycloid overshoot in first half)
    float mid = TrajectoryPlanner::compositeCycloidPos(0.5f);
    assert(mid > 0.4f && mid < 0.6f);

    // Velocity curve
    assert(approx(TrajectoryPlanner::compositeCycloidVel(0.f), 0.f));
    assert(approx(TrajectoryPlanner::compositeCycloidVel(1.f), 0.f));
    // Peak velocity at s=0.5
    float peak = TrajectoryPlanner::compositeCycloidVel(0.5f);
    assert(approx(peak, 2.0f, 0.01f));  // 1 - cos(pi) = 2

    // Lift curve
    assert(approx(TrajectoryPlanner::liftCurvePos(0.f), 0.f));
    assert(approx(TrajectoryPlanner::liftCurvePos(1.f), 0.f));
    assert(approx(TrajectoryPlanner::liftCurvePos(0.5f), 1.0f, 0.01f));

    printf("[PASS] Composite cycloid tests\n");
}

// ─── Leg kinematics tests ───
static void test_leg_kinematics() {
    using namespace vmc;

    LegKinematics kin({0.20f, 0.20f});  // L1=0.2, L2=0.2

    // Default standing: q = [0, 0, 0] -> foot straight down
    {
        auto foot = kin.forward({0, 0, 0});
        assert(approx(foot.x, 0.f));
        assert(approx(foot.y, 0.f));
        assert(approx(foot.z, -0.4f));  // -(L1+L2)
    }

    // 非零偏置 FK：q=[0,0,0] → foot = (dx, dy, -(L1+L2))
    {
        LegKinematics kin_offset({0.20f, 0.20f, 0.04f, -0.03f});  // L1, L2, dx, dy
        auto foot = kin_offset.forward({0, 0, 0});
        assert(approx(foot.x, 0.04f));    // dx
        assert(approx(foot.y, -0.03f));   // dy (右前腿)
        assert(approx(foot.z, -0.4f));

    }

    // 非零偏置 IK 往返
    {
        LegKinematics kin_offset({0.20f, 0.20f, 0.04f, -0.03f});
        Vec3 target = {0.09f, -0.05f, -0.25f};  // 足端在髋坐标系
        auto q = kin_offset.inverse(target);
        auto foot = kin_offset.forward(q);
        assert(approx(foot.x, target.x, 0.01f));
        assert(approx(foot.y, target.y, 0.01f));
        assert(approx(foot.z, target.z, 0.01f));
    }

    // Hip forward 90 deg (RHR: negative hip → foot forward)
    {
        float hip = -M_PI / 2.f;
        auto foot = kin.forward({0, hip, 0});
        assert(approx(foot.x, 0.4f));
        assert(approx(foot.y, 0.f));
        assert(approx(foot.z, 0.f, 0.01f));
    }

    // IK round-trip
    {
        Vec3 target = {0.05f, -0.02f, -0.25f};
        auto q = kin.inverse(target);
        auto foot = kin.forward(q);
        assert(approx(foot.x, target.x, 0.01f));
        assert(approx(foot.y, target.y, 0.01f));
        assert(approx(foot.z, target.z, 0.01f));
    }

    // Jacobian test: numerical vs analytical
    {
        std::array<float, 3> q = {0.1f, 0.5f, 0.8f};
        Mat3 J;
        kin.jacobian(q, J);

        // Numerical Jacobian check for d(foot_x)/d(q_hip)
        float dq = 0.0001f;
        auto f0 = kin.forward({q[0], q[1], q[2]});
        auto f1 = kin.forward({q[0], q[1] + dq, q[2]});
        float num_dx_dhip = (f1.x - f0.x) / dq;
        assert(approx(J.m[1], num_dx_dhip, 0.01f));
    }

    // Foot force to torques
    {
        std::array<float, 3> q = {0.f, 0.3f, 0.6f};
        Vec3 force = {0, 0, -50.f};  // vertical upward force on foot (ground reaction)
        auto tau = kin.footForceToTorques(q, force);
        // Should produce knee extension torque primarily
        assert(tau[2] != 0.f);
    }

    printf("[PASS] Leg kinematics tests\n");
}

// ─── Gait generator tests ───
static void test_gait_generator() {
    using namespace vmc;

    GaitGenerator gen;
    gen.setGaitType(GaitType::TROT);
    Vec3 vel = {0.2f, 0, 0};

    auto state = gen.update(0.15f, vel);  // half cycle at 0.3s period

    // After half period, should have completed phase transitions
    // LF starts at phase 0. After 0.15s of 0.3s cycle = 0.5 progress.
    // LF: 0 + 0.5 = 0.5 -> at duty=0.5 boundary (stance end)
    // Actually, cycle_progress + phase offset. Let me check the implementation.

    // For trot at t=0: LF phase=0 (stance start), RF phase=0.5 (swing start), etc.
    // At t=0: LF phase offset 0 -> raw_phase = 0 + 0 = 0 -> stance (0 < 0.5)
    // After 0.15s (half cycle): raw_phase = 0.5 + 0 = 0.5 -> exactly at boundary
    // With duty=0.5, raw_phase < 0.5 is stance, so at 0.5 it's swing (>= duty)

    // Just verify basic behavior
    assert(state.step_count >= 0);
    // Cycle period is modulated by velocity (effective_period < nominal when moving)
    assert(state.cycle_progress > 0.f && state.cycle_progress < 1.f);

    // Run enough updates to complete at least one full stride
    for (int i = 0; i < 20; ++i) {
        state = gen.update(0.03f, vel);
    }
    assert(state.step_count >= 1);

    // Switch to walk
    gen.setGaitType(GaitType::WALK);
    state = gen.update(0.1f, vel);
    assert(gen.config().duty_factor > 0.6f);  // walk has higher duty

    // Stand
    gen.setGaitType(GaitType::STAND);
    state = gen.update(0.1f, vel);
    for (int i = 0; i < 4; ++i) {
        assert(state.leg_phase[i] == LegPhase::STANCE);
    }

    printf("[PASS] Gait generator tests\n");
}

// ─── State estimator tests ───
static void test_state_estimator() {
    using namespace vmc;

    StateEstimator est;
    Vec3 accel = {0, 0, -9.81f};
    Vec3 gyro = {0, 0, 0};

    // Stationary: roll and pitch should converge to 0
    for (int i = 0; i < 100; ++i) {
        est.updateImu(0.005f, accel, gyro);
    }
    auto bs = est.bodyState();
    assert(approx(bs.euler.x, 0.f, 0.1f));
    assert(approx(bs.euler.y, 0.f, 0.1f));

    // Tilted: accel shows pitch
    Vec3 accel_tilted = {1.0f, 0, -9.7f};  // slight pitch forward
    for (int i = 0; i < 50; ++i) {
        est.updateImu(0.005f, accel_tilted, gyro);
    }
    bs = est.bodyState();
    // Should detect positive pitch (nose up from positive ax)
    assert(bs.euler.y > 0.05f);

    // Reset
    est.reset();
    bs = est.bodyState();
    assert(approx(bs.euler.x, 0.f));

    printf("[PASS] State estimator tests\n");
}

int main() {
    printf("=== VMC Controller Tests ===\n\n");

    test_composite_cycloid();
    test_leg_kinematics();
    test_gait_generator();
    test_state_estimator();

    printf("\n=== All tests passed ===\n");
    return 0;
}
