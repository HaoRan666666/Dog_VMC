"""
Quadruped VMC Position Controller.

Provides composite cycloid trajectory planning, leg kinematics,
gait scheduling, state estimation, and balance control for a
12-DOF quadruped robot.

Usage:
    import vmc_controller_py as vmc

    params = vmc.RobotParams()
    params.thigh_length = 0.20
    params.calf_length = 0.20
    params.body_mass = 5.0

    ctrl = vmc.QuadrupedController(params)
    ctrl.set_config(vmc.GaitType.TROT, step_length=0.06, step_height=0.04,
                    swing_kp=80, swing_kd=5, stance_kp=30, stance_kd=3)

    motion = vmc.RobotMotionCommand()
    motion.target_velocity = vmc.Vec3(0.15, 0.0, 0.0)

    while running:
        ctrl.update_imu(dt, accel_vec, gyro_vec)
        ctrl.set_motion_command(motion)
        commands = ctrl.step(dt)
        # send commands.legs[leg][joint] to motors via MIT mode
"""

from .example_walk import (
    composite_cycloid_pos,
    composite_cycloid_vel,
    lift_curve_pos,
    lift_curve_vel,
    QuadrupedSimulator,
)
