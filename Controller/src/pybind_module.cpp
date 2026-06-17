#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "vmc_controller/types.h"
#include "vmc_controller/trajectory_planner.h"
#include "vmc_controller/leg_kinematics.h"
#include "vmc_controller/gait_generator.h"
#include "vmc_controller/state_estimator.h"
#include "vmc_controller/balance_controller.h"
#include "vmc_controller/quadruped_controller.h"

namespace py = pybind11;
using namespace vmc;

PYBIND11_MODULE(vmc_controller_py, m) {
    m.doc() = "Quadruped VMC position controller with composite cycloid trajectories";

    // ─── Enums ───
    py::enum_<LegIndex>(m, "LegIndex")
        .value("LF", LegIndex::LF)
        .value("RF", LegIndex::RF)
        .value("LB", LegIndex::LB)
        .value("RB", LegIndex::RB)
        .export_values();

    py::enum_<JointIndex>(m, "JointIndex")
        .value("ABD", JointIndex::ABD)
        .value("HIP", JointIndex::HIP)
        .value("KNEE", JointIndex::KNEE)
        .export_values();

    py::enum_<GaitType>(m, "GaitType")
        .value("TROT", GaitType::TROT)
        .value("WALK", GaitType::WALK)
        .value("BOUND", GaitType::BOUND)
        .value("PACE", GaitType::PACE)
        .value("STAND", GaitType::STAND)
        .export_values();

    py::enum_<LegPhase>(m, "LegPhase")
        .value("STANCE", LegPhase::STANCE)
        .value("SWING", LegPhase::SWING)
        .export_values();

    // ─── Vec3 ───
    py::class_<Vec3>(m, "Vec3")
        .def(py::init<>())
        .def(py::init<float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &Vec3::x)
        .def_readwrite("y", &Vec3::y)
        .def_readwrite("z", &Vec3::z)
        .def("__add__", [](const Vec3& a, const Vec3& b) { return a + b; })
        .def("__sub__", [](const Vec3& a, const Vec3& b) { return a - b; })
        .def("__mul__", [](const Vec3& v, float s) { return v * s; })
        .def("norm", &Vec3::norm)
        .def("normalized", &Vec3::normalized)
        .def("__repr__", [](const Vec3& v) {
            return "Vec3(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        });

    // ─── RobotParams ───
    py::class_<RobotParams>(m, "RobotParams")
        .def(py::init<>())
        .def_readwrite("body_length", &RobotParams::body_length)
        .def_readwrite("body_width", &RobotParams::body_width)
        .def_readwrite("body_height", &RobotParams::body_height)
        .def_readwrite("thigh_length", &RobotParams::thigh_length)
        .def_readwrite("calf_length", &RobotParams::calf_length)
        .def_readwrite("body_mass", &RobotParams::body_mass)
        .def_readwrite("nominal_stand_height", &RobotParams::nominal_stand_height)
        .def_readwrite("abd_min", &RobotParams::abd_min)
        .def_readwrite("abd_max", &RobotParams::abd_max)
        .def_readwrite("hip_min", &RobotParams::hip_min)
        .def_readwrite("hip_max", &RobotParams::hip_max)
        .def_readwrite("knee_min", &RobotParams::knee_min)
        .def_readwrite("knee_max", &RobotParams::knee_max)
        .def_readwrite("hip_offset_x_front", &RobotParams::hip_offset_x_front)
        .def_readwrite("hip_offset_x_rear", &RobotParams::hip_offset_x_rear)
        .def_readwrite("hip_offset_y", &RobotParams::hip_offset_y)
        .def_readwrite("hip_offset_z", &RobotParams::hip_offset_z)
        .def_readwrite("hip_dx", &RobotParams::hip_dx)
        .def_readwrite("hip_dy", &RobotParams::hip_dy)
        .def_property("joint_zero_offset",
            [](const RobotParams& p) { return p.joint_zero_offset; },
            [](RobotParams& p, const std::array<std::array<float, 3>, 4>& v) {
                p.joint_zero_offset = v;
            });

    // ─── BodyState ───
    py::class_<BodyState>(m, "BodyState")
        .def(py::init<>())
        .def_readwrite("position", &BodyState::position)
        .def_readwrite("velocity", &BodyState::velocity)
        .def_readwrite("euler", &BodyState::euler)
        .def_readwrite("angular_velocity", &BodyState::angular_velocity)
        .def_readwrite("linear_accel", &BodyState::linear_accel);

    // ─── GaitState ───
    py::class_<GaitState>(m, "GaitState")
        .def(py::init<>())
        .def_readonly("cycle_progress", &GaitState::cycle_progress)
        .def_readonly("step_count", &GaitState::step_count)
        .def("__getitem__", [](const GaitState& gs, int leg) {
            return std::make_tuple(gs.leg_phase[leg], gs.phase[leg]);
        });

    // ─── RobotMotionCommand ───
    py::class_<RobotMotionCommand>(m, "RobotMotionCommand")
        .def(py::init<>())
        .def_readwrite("target_velocity", &RobotMotionCommand::target_velocity)
        .def_readwrite("target_yaw_rate", &RobotMotionCommand::target_yaw_rate)
        .def_readwrite("target_body_height", &RobotMotionCommand::target_body_height)
        .def_readwrite("target_euler", &RobotMotionCommand::target_euler);

    // ─── MotorCommand ───
    py::class_<MotorCommand>(m, "MotorCommand")
        .def(py::init<>())
        .def_readwrite("position", &MotorCommand::position)
        .def_readwrite("velocity", &MotorCommand::velocity)
        .def_readwrite("kp", &MotorCommand::kp)
        .def_readwrite("kd", &MotorCommand::kd)
        .def_readwrite("feedforward_torque", &MotorCommand::feedforward_torque)
        .def("__repr__", [](const MotorCommand& c) {
            return "MotorCmd(pos=" + std::to_string(c.position) +
                   ", vel=" + std::to_string(c.velocity) +
                   ", kp=" + std::to_string(c.kp) +
                   ", kd=" + std::to_string(c.kd) +
                   ", ff=" + std::to_string(c.feedforward_torque) + ")";
        });

    // ─── RobotControlCommand ───
    py::class_<RobotControlCommand>(m, "RobotControlCommand")
        .def(py::init<>())
        .def("get", [](const RobotControlCommand& c, int leg, int joint) {
            return c.legs[leg][joint];
        })
        .def("__getitem__", [](const RobotControlCommand& c, int leg) {
            return c.legs[leg];
        })
        .def("__len__", [](const RobotControlCommand&) { return 4; });

    // ─── JointState / RobotJointState ───
    py::class_<JointState>(m, "JointState")
        .def(py::init<>())
        .def_readwrite("position", &JointState::position)
        .def_readwrite("velocity", &JointState::velocity)
        .def_readwrite("torque", &JointState::torque);

    py::class_<RobotJointState>(m, "RobotJointState")
        .def(py::init<>())
        .def_readwrite("legs", &RobotJointState::legs);

    // ─── TrajectoryPlanner ───
    py::class_<TrajectoryPlanner>(m, "TrajectoryPlanner")
        .def(py::init<>())
        .def_static("composite_cycloid_pos", &TrajectoryPlanner::compositeCycloidPos)
        .def_static("composite_cycloid_vel", &TrajectoryPlanner::compositeCycloidVel)
        .def_static("lift_curve_pos", &TrajectoryPlanner::liftCurvePos)
        .def_static("lift_curve_vel", &TrajectoryPlanner::liftCurveVel);

    // ─── LegKinematics ───
    py::class_<LegKinematics>(m, "LegKinematics")
        .def(py::init<>())
        .def(py::init([](float L1, float L2, float dx, float dy) {
            LegKinematics::Params p;
            p.L1 = L1; p.L2 = L2; p.dx = dx; p.dy = dy;
            return std::make_unique<LegKinematics>(p);
        }), py::arg("L1")=0.2f, py::arg("L2")=0.2f,
            py::arg("dx")=0.0f, py::arg("dy")=0.0f)
        .def("forward", [](const LegKinematics& kin, const std::array<float, 3>& q) {
            return kin.forward(q);
        })
        .def("inverse", [](const LegKinematics& kin, const Vec3& pos) {
            return kin.inverse(pos);
        })
        .def("foot_force_to_torques", [](const LegKinematics& kin,
            const std::array<float, 3>& q, const Vec3& force) {
            return kin.footForceToTorques(q, force);
        });

    // ─── GaitGenerator ───
    py::class_<GaitGenerator>(m, "GaitGenerator")
        .def(py::init<>())
        .def("set_gait_type", &GaitGenerator::setGaitType)
        .def("update", &GaitGenerator::update)
        .def("get_swing_progress", &GaitGenerator::getSwingProgress)
        .def("get_stance_progress", &GaitGenerator::getStanceProgress)
        .def_property_readonly("state", &GaitGenerator::state)
        .def_static("nominal_foot_position", &GaitGenerator::nominalFootPosition)
        .def_static("hip_offset", &GaitGenerator::hipOffset);

    // ─── StateEstimator ───
    py::class_<StateEstimator>(m, "StateEstimator")
        .def(py::init<>())
        .def("update_imu", &StateEstimator::updateImu)
        .def_property_readonly("body_state", &StateEstimator::bodyState)
        .def("reset", &StateEstimator::reset);

    // ─── QuadrupedController (main entry point) ───
    py::class_<QuadrupedController>(m, "QuadrupedController")
        .def(py::init<const RobotParams&>(), py::arg("params") = RobotParams{})
        .def("set_config", [](QuadrupedController& ctrl,
            GaitType gait, float step_length, float step_height,
            float swing_kp, float swing_kd, float stance_kp, float stance_kd) {
            QuadrupedController::Config cfg;
            cfg.gait_type = gait;
            cfg.step_length = step_length;
            cfg.step_height = step_height;
            cfg.swing_kp = swing_kp;
            cfg.swing_kd = swing_kd;
            cfg.stance_kp = stance_kp;
            cfg.stance_kd = stance_kd;
            ctrl.setConfig(cfg);
        })
        .def("set_motion_command", &QuadrupedController::setMotionCommand)
        .def("update_imu", &QuadrupedController::updateImu)
        .def("set_joint_feedback", &QuadrupedController::setJointFeedback)
        .def("set_joint_feedback_raw", &QuadrupedController::setJointFeedbackRaw)
        .def("step", &QuadrupedController::step)
        // ─── 零点标定 ───
        .def("calibrate_joint", &QuadrupedController::calibrateJoint,
             py::arg("leg"), py::arg("joint"),
             py::arg("motor_reading"), py::arg("kinematic_angle"))
        .def("calibrate_from_reference_pose",
             &QuadrupedController::calibrateFromReferencePose)
        .def("set_joint_zero_offsets", &QuadrupedController::setJointZeroOffsets)
        .def("joint_zero_offsets", &QuadrupedController::jointZeroOffsets)
        .def("raw_to_kinematic", &QuadrupedController::rawToKinematic,
             py::arg("leg"), py::arg("joint"), py::arg("raw_angle"))
        .def("kinematic_to_raw", &QuadrupedController::kinematicToRaw,
             py::arg("leg"), py::arg("joint"), py::arg("kin_angle"))
        // ─── 状态访问 ───
        .def_property_readonly("body_state", &QuadrupedController::bodyState)
        .def_property_readonly("gait_state", &QuadrupedController::gaitState)
        .def_property_readonly("robot_params", &QuadrupedController::robotParams);
}
