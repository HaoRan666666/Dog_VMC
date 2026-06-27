/**
 * 四足机器人遥控控制节点 (C++)
 *
 * 依赖: kinematics(运动学), motors(电机), rclcpp, sensor_msgs
 */

#include "kinematics/leg_kinematics.h"
#include "kinematics/types.h"

#include <motor_driver.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace vmc;
using Joy = sensor_msgs::msg::Joy;

// ════════════════════════════════════
// 常量
// ════════════════════════════════════

// ── 手柄映射（Xbox + ROS2 joy_node） ──
constexpr int AXIS_LX=0, AXIS_LY=1, AXIS_LT=2;              // 左摇杆 X/Y, 左扳机
constexpr int AXIS_RX=3, AXIS_RT=5, AXIS_DPAD_Y=7;          // 右摇杆 X, 右扳机, 十字键上下
constexpr int BTN_A=0, BTN_B=1, BTN_RB=5;                   // A/B/RB 按钮

// ── 运动学（摘自 URDF，与 test_quad_trot 一致） ──
constexpr float L1 = 0.2125f;       // 大腿长度 (m)
constexpr float L2 = 0.25025f;      // 小腿长度 (m)
constexpr float HIP_DX = 0.06f;     // ABD→HIP 前后偏置 (m)
constexpr float HIP_DY = 0.082f;    // ABD→HIP 左右偏置 (m)

// ── 机身几何 ──
constexpr float BODY_LEN = 0.42f;   // 前后髋间距 (m)
constexpr float BODY_WID = 0.154f;  // 左右髋间距 (m)
constexpr float HIP_COM_X[4] = { BODY_LEN/2, BODY_LEN/2, -BODY_LEN/2, -BODY_LEN/2 };
constexpr float HIP_COM_Y[4] = { BODY_WID/2, -BODY_WID/2, BODY_WID/2, -BODY_WID/2 };
constexpr float FOOT_X_OFF[4] = {0, 0, -0.03f, -0.03f};  // 后腿足端后移 (m)

// ── PD 控制参数 ──
constexpr float KP_STAND=100.f, KD_STAND=5.f;    // 站立/支撑相
constexpr float KP_SWING=100.f, KD_SWING=5.f;    // 摆动相
constexpr float KP_TRANS=100.f, KD_TRANS=5.f;    // S 曲线过渡

// ── 高度参数（足端 Z，负=向下，髋关节坐标系） ──
constexpr float CROUCH_Z = -0.15f;        // 趴下 (m)
constexpr float DEFAULT_STAND_Z = -0.2f; // 默认站立 (m)
constexpr float MAX_STAND_Z = -0.38f;     // 最高站立 (m)
constexpr float HEIGHT_SMOOTH = 0.03f;    // 高度平滑速率 (m/s)

// ── Trot 步态参数 ──
constexpr float TROT_CYCLE = 0.8f;        // 步态周期 (s)
constexpr float TROT_DUTY = 0.60f;        // 支撑相占比
constexpr float TROT_MAX_STEP = 0.15f;    // 最大步长 (m)
constexpr float TROT_LATERAL = 0.06f;     // 最大横移幅度 (m)
constexpr float TROT_YAW = 0.04f;         // 最大偏航幅度 (m)
constexpr float TROT_STEP_H = 0.04f;      // 抬腿高度 (m)
constexpr float TROT_START_T = TROT_DUTY * TROT_CYCLE / 2.f;  // 起始相位 (四腿在轨迹中心)

// ── 时序 ──
constexpr float CMD_TIMEOUT = 0.3f;       // 手柄超时保护 (s)
constexpr float STAND_UP_T = 2.0f;        // 趴下→站立过渡时间 (s)
constexpr float LIE_DOWN_T = 2.5f;        // 站立→趴下过渡时间 (s)
constexpr float DT = 0.005f;              // 控制周期 200Hz (s)

enum State { DISABLED = 0, CROUCH, STANDING, TROTTING };

// 电机映射
const std::map<int, std::tuple<int,int,int,std::string>> MOTORS = {
    {0, {1,2,3,"can0"}}, {1, {1,2,3,"can1"}}, {2, {1,2,3,"can2"}}, {3, {1,2,3,"can3"}}
};

// 零点 + 方向 (同 test_quad_trot)
const std::array<std::array<float,3>,4> ZERO = {{
    {{0,0, 0.8111f}}, {{0,0,-0.8111f}}, {{0,0, 0.8111f}}, {{0,0,-0.8111f}}
}};
const std::array<std::array<float,3>,4> JSIGN = {{
    {{ 1, 1, 1}}, {{ 1,-1,-1}}, {{-1, 1, 1}}, {{-1,-1,-1}}
}};
const std::array<float,4> PHASE_OFF = {0.f, 0.5f, 0.5f, 0.f};

// ════════════════════════════════════
// 辅助
// ════════════════════════════════════

static float dead_zone(float v, float dz = 0.08f) { return std::abs(v) < dz ? 0.f : v; }
static float cycloid(float s) { return s <= 0 ? 0.f : s >= 1 ? 1.f : s - std::sin(2*M_PI*s)/(2*M_PI); }
static float lift(float s) { return s <= 0 || s >= 1 ? 0.f : 0.5f*(1-std::cos(2*M_PI*s)); }

static LegKinematics make_kin(int leg) {
    float xs = (leg < 2) ? 1.f : -1.f;
    float ys = (leg == 0 || leg == 2) ? 1.f : -1.f;
    LegKinematics::Params p; p.L1 = L1; p.L2 = L2; p.dx = xs*HIP_DX; p.dy = ys*HIP_DY;
    return LegKinematics(p);
}

static std::array<float,3> ik(LegKinematics& kin, float dx, float dy, float z) {
    return kin.inverse(Vec3(dx, dy, z));
}

// ════════════════════════════════════
// 控制节点
// ════════════════════════════════════

class QuadControl : public rclcpp::Node {
public:
    QuadControl() : Node("quad_control") {
        // 运动学
        for (int i = 0; i < 4; ++i) {
            kin_[i] = make_kin(i);
            float xs = (i < 2) ? 1.f : -1.f, ys = (i == 0 || i == 2) ? 1.f : -1.f;
            dx_[i] = xs * HIP_DX; dy_[i] = ys * HIP_DY;
        }
        foot_z_ = target_z_ = CROUCH_Z; saved_z_ = DEFAULT_STAND_Z;

        // 电机
        try { init_motors(); }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "电机初始化失败: %s（键盘/仿真模式下忽略）", e.what());
            motors_ok_ = false;
        }

        // ROS2
        joy_sub_ = this->create_subscription<Joy>("/joy", 10,
            std::bind(&QuadControl::on_joy, this, std::placeholders::_1));
        ctrl_timer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int>(DT*1e6)),
            std::bind(&QuadControl::control_loop, this));

        RCLCPP_INFO(get_logger(), "四足控制节点就绪  RB=使能 B=站立/趴下 A=Trot RT/LT=高度");
    }

    ~QuadControl() { disable(); deinit_motors(); }

private:
    // ── 电机 ──
    std::map<std::pair<int,int>, std::shared_ptr<MotorDriver>> motors_;

    void init_motors() {
        for (auto& [leg, ids] : MOTORS) {
            int a = std::get<0>(ids), h = std::get<1>(ids), k = std::get<2>(ids);
            auto& can = std::get<3>(ids);
            for (auto [j, id] : {std::pair{0,a}, {1,h}, {2,k}}) {
                auto m = MotorDriver::create_motor(id, "CAN", can, "LRO_CAN", 2);
                m->init_motor();
                motors_[{leg,j}] = m;
            }
        }
    }
    void deinit_motors() { for (auto& [_, m] : motors_) m->deinit_motor(); }
    void lock_all()   { for (auto& [_, m] : motors_) m->lock_motor(); }
    void unlock_all() { for (auto& [_, m] : motors_) m->unlock_motor(); }

    std::array<float,3> read_q(int leg) {
        std::array<float,3> q{};
        for (int j = 0; j < 3; ++j) {
            auto& m = motors_.at({leg,j});
            m->refresh_motor_status();
            q[j] = JSIGN[leg][j] * (m->get_motor_pos() - ZERO[leg][j]);
        }
        return q;
    }

    void send_mit(int leg, const std::array<float,3>& q, float kp, float kd,
                  const std::array<float,3>& ff = {0,0,0}) {
        for (int j = 0; j < 3; ++j) {
            auto& m = motors_.at({leg,j});
            float raw = ZERO[leg][j] + JSIGN[leg][j] * q[j];
            float fm = JSIGN[leg][j] * ff[j];
            m->motor_mit_cmd(raw, 0.f, kp, kd, fm);
        }
    }

    // ── 运动学 ──
    LegKinematics kin_[4];
    float dx_[4], dy_[4];
    float foot_cx(int leg) { return dx_[leg] + FOOT_X_OFF[leg]; }

    // ── 状态 ──
    bool motors_ok_ = true;
    State state_ = DISABLED;
    float foot_z_ = DEFAULT_STAND_Z, target_z_ = CROUCH_Z, saved_z_ = DEFAULT_STAND_Z;
    float step_h_ = TROT_STEP_H;
    float trot_t_ = TROT_START_T;
    float trot_step_ = 0, trot_lat_ = 0, trot_yaw_ = 0;
    bool pending_stand_ = false, transition_active_ = false;
    std::atomic<bool> running_{true};

    std::map<int,int> last_btn_;
    float last_dpad_ = 0;
    rclcpp::Time last_joy_time_;

    // ── ROS2 ──
    rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_;

    // ══════════════════════
    // 手柄回调
    // ══════════════════════

    bool btn_rising(int idx, const std::vector<int>& btns) {
        int prev = last_btn_.count(idx) ? last_btn_[idx] : 0;
        return idx < (int)btns.size() && btns[idx] && !prev;
    }

    float axis(const std::vector<float>& ax, int idx) {
        return idx < (int)ax.size() ? ax[idx] : 0.f;
    }

    void on_joy(const Joy::SharedPtr msg) {
        last_joy_time_ = now();
        auto& btns = msg->buttons;
        auto& ax = msg->axes;

        if (btn_rising(BTN_RB, btns)) {
            if (state_ == DISABLED) enable(); else disable();
        }
        if (state_ == DISABLED) { update_btns(btns); return; }

        if (btn_rising(BTN_B, btns)) {
            if (state_ == CROUCH) transition_to(STANDING);
            else if (state_ == STANDING) transition_to(CROUCH);
        }
        if (btn_rising(BTN_A, btns)) {
            if (state_ == CROUCH || state_ == STANDING) transition_to(TROTTING);
            else if (state_ == TROTTING) {
                if (std::abs(trot_step_) < 0.005f && std::abs(trot_lat_) < 0.005f
                    && std::abs(trot_yaw_) < 0.005f)
                    pending_stand_ = true;
                else RCLCPP_WARN(get_logger(), "请先松开摇杆再按 A");
            }
        }

        // 摇杆
        if (state_ == TROTTING) {
            float lx = dead_zone(axis(ax, AXIS_LX));
            float ly = dead_zone(axis(ax, AXIS_LY));
            float rx = dead_zone(axis(ax, AXIS_RX));
            trot_step_ = TROT_MAX_STEP * std::abs(ly);
            if (ly < 0) trot_step_ = -trot_step_;
            trot_lat_ = lx * TROT_LATERAL;
            trot_yaw_ = rx * TROT_YAW;
            if (pending_stand_ && (std::abs(trot_step_) > 0.005f
                || std::abs(trot_lat_) > 0.005f || std::abs(trot_yaw_) > 0.005f)) {
                pending_stand_ = false;
                RCLCPP_INFO(get_logger(), "摇杆离开中位，取消站立切换");
            }
        } else { trot_step_ = trot_lat_ = trot_yaw_ = 0; }

        // 高度
        if (state_ == STANDING) {
            float rt = std::max(0.f, axis(ax, AXIS_RT));
            float lt = std::max(0.f, axis(ax, AXIS_LT));
            target_z_ += (lt - rt) * 0.0008f;
            target_z_ = std::max(MAX_STAND_Z, std::min(CROUCH_Z, target_z_));
        }

        // 十字键
        if (state_ == CROUCH || state_ == STANDING) {
            float dp = axis(ax, AXIS_DPAD_Y);
            if (dp > 0.5f && last_dpad_ <= 0.5f) { step_h_ = std::min(0.1f, step_h_+0.01f); }
            else if (dp < -0.5f && last_dpad_ >= -0.5f) { step_h_ = std::max(0.01f, step_h_-0.01f); }
            last_dpad_ = dp;
        }
        update_btns(btns);
    }

    void update_btns(const std::vector<int>& btns) {
        last_btn_.clear(); for (size_t i = 0; i < btns.size(); ++i) last_btn_[i] = btns[i];
    }

    // ══════════════════════
    // 状态机
    // ══════════════════════

    void enable() {
        RCLCPP_INFO(get_logger(), "RB: 使能, 过渡到趴下...");
        target_z_ = foot_z_ = CROUCH_Z;
        transition_active_ = true;
        std::thread([this]() {
            state_ = CROUCH;
            s_curve_to_z(LIE_DOWN_T);
            transition_active_ = false;
            RCLCPP_INFO(get_logger(), "  趴下就绪");
        }).detach();
    }

    void disable() {
        RCLCPP_INFO(get_logger(), "RB: 失能!");
        state_ = DISABLED;
        target_z_ = foot_z_ = CROUCH_Z;
        trot_step_ = trot_lat_ = trot_yaw_ = 0;
        pending_stand_ = false;
        unlock_all();
    }

    void transition_to(State s) {
        if (transition_active_) return;
        transition_active_ = true;
        std::thread([this, s]() { do_transition(s); }).detach();
    }

    void do_transition(State s) {
        State old = state_;
        if (s == CROUCH) {
            target_z_ = foot_z_ = CROUCH_Z;
            s_curve_to_z(LIE_DOWN_T);
            state_ = CROUCH;
        } else if (s == STANDING) {
            if (old != TROTTING) { target_z_ = saved_z_; foot_z_ = saved_z_; }
            s_curve_to_z(STAND_UP_T);
            state_ = STANDING;
        } else if (s == TROTTING) {
            if (old == STANDING) saved_z_ = foot_z_;
            if (foot_z_ != target_z_) target_z_ = foot_z_;
            s_curve_to_z(0.5f);
            trot_t_ = TROT_START_T;
            trot_step_ = trot_lat_ = trot_yaw_ = 0;
            state_ = TROTTING;
        }
        transition_active_ = false;
    }

    // ══════════════════════
    // S 曲线过渡
    // ══════════════════════

    void s_curve_to_z(float dur) {
        // 读每条腿当前角
        std::array<std::array<float,3>,4> q0;
        std::array<std::array<float,3>,4> qt;
        for (int i = 0; i < 4; ++i) {
            q0[i] = read_q(i);
            qt[i] = ik(kin_[i], foot_cx(i), dy_[i], target_z_);
        }
        int steps = static_cast<int>(dur / DT);
        for (int k = 0; k <= steps; ++k) {
            if (state_ == DISABLED) return;
            float a = 0.5f - 0.5f * std::cos(M_PI * (k+1) / (steps+1));
            foot_z_ = target_z_;
            for (int i = 0; i < 4; ++i) {
                std::array<float,3> q;
                for (int j = 0; j < 3; ++j) q[j] = q0[i][j] + a * (qt[i][j] - q0[i][j]);
                send_mit(i, q, KP_TRANS, KD_TRANS);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(DT*1e6)));
        }
    }

    // ══════════════════════
    // 控制循环
    // ══════════════════════

    void control_loop() {
        if (!motors_ok_ || state_ == DISABLED || transition_active_) return;

        // 通信超时保护
        if ((now() - last_joy_time_).seconds() > CMD_TIMEOUT) {
            if (state_ == TROTTING) { trot_step_ = trot_lat_ = trot_yaw_ = 0; }
        }

        // 高度平滑
        if (state_ == CROUCH || state_ == STANDING) {
            float dz = std::abs(foot_z_ - target_z_);
            if (dz > 1e-4f) {
                float step = HEIGHT_SMOOTH * DT;
                if (target_z_ > foot_z_) foot_z_ = std::min(foot_z_+step, target_z_);
                else foot_z_ = std::max(foot_z_-step, target_z_);
            }
        }

        if (state_ == CROUCH || state_ == STANDING) {
            for (int i = 0; i < 4; ++i) {
                send_mit(i, ik(kin_[i], foot_cx(i), dy_[i], foot_z_), KP_STAND, KD_STAND);
            }
        } else if (state_ == TROTTING) {
            if (pending_stand_ && all_in_stance()) {
                pending_stand_ = false;
                RCLCPP_INFO(get_logger(), "四腿着地, 切换站立");
                transition_to(STANDING);
            } else {
                trot_step();
            }
        }
    }

    // ══════════════════════
    // Trot 步态
    // ══════════════════════

    bool all_in_stance() {
        for (int i = 0; i < 4; ++i)
            if (std::fmod(trot_t_/TROT_CYCLE + PHASE_OFF[i], 1.f) >= TROT_DUTY) return false;
        return true;
    }

    float cycloid_disp(float step, float ps, bool stance) {
        return stance ? step*0.5f - step*ps : -step*0.5f + step*cycloid(ps);
    }

    void trot_step() {
        float sx = trot_step_, sy = trot_lat_, yaw = trot_yaw_;

        for (int i = 0; i < 4; ++i) {
            float raw_p = std::fmod(trot_t_/TROT_CYCLE + PHASE_OFF[i], 1.f);
            bool stance = raw_p < TROT_DUTY;
            float ps = stance ? raw_p/TROT_DUTY : (raw_p-TROT_DUTY)/(1.f-TROT_DUTY);

            float xo = cycloid_disp(sx, ps, stance);
            float yo = cycloid_disp(sy, ps, stance);

            if (std::abs(yaw) > 1e-6f) {
                float fx = HIP_COM_X[i] + foot_cx(i);
                float fy = HIP_COM_Y[i] + dy_[i];
                float tx = -fy, ty = fx, n = std::sqrt(tx*tx+ty*ty);
                if (n > 1e-9f) { tx/=n; ty/=n; }
                float yd = cycloid_disp(yaw, ps, stance);
                xo += tx * yd; yo += ty * yd;
            }

            float z = stance ? foot_z_ : foot_z_ + step_h_ * lift(ps);
            auto q = kin_[i].inverse(Vec3(foot_cx(i)+xo, dy_[i]+yo, z));
            send_mit(i, q, stance ? KP_STAND : KP_SWING, stance ? KD_STAND : KD_SWING);
        }
        trot_t_ += DT;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadControl>());
    rclcpp::shutdown();
    return 0;
}
