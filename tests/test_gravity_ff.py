#!/usr/bin/env python3
"""
单腿重力前馈测试：验证 Jᵀ·F 方向与量级。

悬空测试 → 开启前馈 → 观察腿是伸长还是收拢。
正确行为：FF 模拟地面推脚向上，电机推脚向下 → 腿轻微伸长，PD 自动拉回。

用法：
  python3 tests/test_gravity_ff.py --leg 0        # LF 腿
  python3 tests/test_gravity_ff.py --leg 1        # RF 腿
  python3 tests/test_gravity_ff.py --leg all --mass 16.0
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'kinematics', 'build'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Module', 'motors', 'build'))

import kinematics_py as vmc

try:
    import motors_py
    HAS_MOTORS = True
except ImportError:
    HAS_MOTORS = False

# ── 运动学参数 ──
L1, L2 = 0.2125, 0.25025
HIP_DX, HIP_DY = 0.06, 0.082
G = 9.81

MOTOR_MAP = {
    0: (1, 2, 3, "can0"),
    1: (1, 2, 3, "can1"),
    2: (1, 2, 3, "can2"),
    3: (1, 2, 3, "can3"),
}

ZERO_OFFSET = {
    0: [0.0, 0.0,  0.8111], 1: [0.0, 0.0, -0.8111],
    2: [0.0, 0.0,  0.8111], 3: [0.0, 0.0, -0.8111],
}

JOINT_SIGN = {
    0: [ 1.0,  1.0,  1.0], 1: [ 1.0, -1.0, -1.0],
    2: [-1.0,  1.0,  1.0], 3: [-1.0, -1.0, -1.0],
}

LEG_NAMES = {0: "LF", 1: "RF", 2: "LB", 3: "RB"}
JOINT_NAMES = {0: "ABD", 1: "HIP", 2: "KNEE"}


def init_motors(leg, sim):
    motors = {}
    if sim:
        return motors
    mid_abd, mid_hip, mid_knee, can_if = MOTOR_MAP[leg]
    for j, mid in enumerate([mid_abd, mid_hip, mid_knee]):
        m = motors_py.MotorDriver.create_motor(mid, "CAN", can_if, "LRO_CAN", 2)
        m.init_motor()
        motors[j] = m
    return motors


def read_q(motors, leg):
    """读取单腿运动学关节角。"""
    q = [0.0, 0.0, 0.0]
    for j in range(3):
        m = motors.get(j)
        if m:
            m.refresh_motor_status()
            raw = m.get_motor_pos()
            q[j] = JOINT_SIGN[leg][j] * (raw - ZERO_OFFSET[leg][j])
    return q

def send_mit(motors, leg, q_kin, kp, kd, ff):
    for j in range(3):
        m = motors.get(j)
        if m:
            raw = ZERO_OFFSET[leg][j] + JOINT_SIGN[leg][j] * q_kin[j]
            fm = JOINT_SIGN[leg][j] * ff[j]  # kinematic → motor torque
            m.motor_mit_cmd(raw, 0.0, kp, kd, fm)

import math as _math
def s_curve_to_stand(motors, leg, q_target, duration, kp, kd, ff):
    """从当前关节角 S 曲线过渡到目标。"""
    q_start = read_q(motors, leg)
    dt = 0.005
    steps = int(duration / dt)
    for i in range(steps + 1):
        a = 0.5 - 0.5 * _math.cos(_math.pi * (i + 1) / (steps + 1))
        q = [q_start[j] + a * (q_target[j] - q_start[j]) for j in range(3)]
        send_mit(motors, leg, q, kp, kd, ff)
        time.sleep(dt)


def compute_ff(leg, mass, foot_z):
    x_sign = 1.0 if leg in (0, 1) else -1.0
    y_sign = 1.0 if leg in (0, 2) else -1.0
    dx, dy = x_sign * HIP_DX, y_sign * HIP_DY
    kin = vmc.LegKinematics(L1, L2, dx, dy)
    q = kin.inverse(vmc.Vec3(dx, dy, foot_z))
    fz = mass * G / 4.0
    tau = kin.foot_force_to_torques(q, vmc.Vec3(0.0, 0.0, fz))
    return q, [float(tau[i]) for i in range(3)], fz


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--leg", type=str, default="0")
    parser.add_argument("--mass", type=float, default=16.0)
    parser.add_argument("--foot_z", type=float, default=-0.25)
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--hold", type=float, default=5.0, help="测试持续时间(s)")
    args = parser.parse_args()

    if args.leg == "all":
        legs = [0, 1, 2, 3]
    else:
        legs = [int(args.leg)]

    print(f"{'='*65}")
    print(f"重力前馈测试  mass={args.mass}kg  Z={args.foot_z}m  legs={legs}")
    print(f"{'='*65}")

    # ── 计算 FF ──
    for leg in legs:
        q, tau, fz = compute_ff(leg, args.mass, args.foot_z)
        tau_motor = [JOINT_SIGN[leg][j] * tau[j] for j in range(3)]
        print(f"\n{LEG_NAMES[leg]}: Fz={fz:.2f}N/leg")
        print(f"  q   = [{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}] rad")
        print(f"  τ_kin = [{tau[0]:+.3f}, {tau[1]:+.3f}, {tau[2]:+.3f}] Nm  (运动学空间)")
        print(f"  τ_mot = [{tau_motor[0]:+.3f}, {tau_motor[1]:+.3f}, {tau_motor[2]:+.3f}] Nm  (电机空间)")
        print(f"  JOINT_SIGN = {JOINT_SIGN[leg]}")

    if args.sim or not HAS_MOTORS:
        print("\n(仿真模式，不驱动电机)")
        return

    # ── 实机测试 ──
    input("\n按 Enter 开始实机测试...")

    all_motors = {}
    for leg in legs:
        all_motors[leg] = init_motors(leg, args.sim)

    kp, kd = 80.0, 3.0

    # S 曲线过渡到站立姿态
    for leg in legs:
        q_stand, _, _ = compute_ff(leg, args.mass, args.foot_z)
        print(f"\n{LEG_NAMES[leg]}: S 曲线过渡到站立...")
        s_curve_to_stand(all_motors[leg], leg, q_stand, 1.5, kp, kd, [0,0,0])

    print("\n站立保持中，按 Enter 开启/关闭前馈，Ctrl+C 退出...")
    try:
        import threading
        ff_flag = [False]
        stop = [False]

        def wait_key():
            while not stop[0]:
                input()
                ff_flag[0] = not ff_flag[0]
                print(f">>> 重力前馈: {'ON' if ff_flag[0] else 'OFF'} <<<")

        threading.Thread(target=wait_key, daemon=True).start()
        t0 = time.time()
        while time.time() - t0 < args.hold and not stop[0]:
            for leg in legs:
                q, tau_ff, _ = compute_ff(leg, args.mass, args.foot_z)
                ff = tau_ff if ff_flag[0] else [0.0, 0.0, 0.0]
                send_mit(all_motors[leg], leg, q, kp, kd, ff)
            time.sleep(0.005)
        stop[0] = True

    except KeyboardInterrupt:
        print("\n中断")
    finally:
        for leg in legs:
            for m in all_motors[leg].values():
                m.lock_motor()
                m.deinit_motor()
        print("结束")


if __name__ == "__main__":
    main()
