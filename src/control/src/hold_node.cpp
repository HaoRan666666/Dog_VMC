/**
 * 电机保持节点 — 读取当前位置并锁住
 */

#include "kinematics/leg_kinematics.h"
#include "kinematics/types.h"

#include <motor_driver.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <map>
#include <memory>
#include <string>
#include <tuple>

using namespace vmc;

// ── 运动学 ──
constexpr float L1 = 0.2125f, L2 = 0.25025f, HIP_DX = 0.06f, HIP_DY = 0.082f;
constexpr float KP = 80.f, KD = 3.f;
constexpr float DT = 0.005f;  // 200Hz

// 电机映射
const std::map<int, std::tuple<int,int,int,std::string>> MOTORS = {
    {0, {1,2,3,"can0"}}, {1, {1,2,3,"can1"}}, {2, {1,2,3,"can2"}}, {3, {1,2,3,"can3"}}
};

const std::array<std::array<float,3>,4> ZERO = {{
    {{0,0, 0.8111f}}, {{0,0,-0.8111f}}, {{0,0, 0.8111f}}, {{0,0,-0.8111f}}
}};
const std::array<std::array<float,3>,4> JSIGN = {{
    {{ 1, 1, 1}}, {{ 1,-1,-1}}, {{-1, 1, 1}}, {{-1,-1,-1}}
}};

class HoldNode : public rclcpp::Node {
public:
    HoldNode() : Node("hold_node") {
        // 初始化电机
        for (auto& [leg, ids] : MOTORS) {
            int a = std::get<0>(ids), h = std::get<1>(ids), k = std::get<2>(ids);
            auto& can = std::get<3>(ids);
            for (auto [j, id] : {std::pair{0,a}, {1,h}, {2,k}}) {
                auto m = MotorDriver::create_motor(id, "CAN", can, "LRO_CAN", 2);
                m->init_motor();
                motors_[{leg,j}] = m;
            }
        }

        // 读取当前位置作为目标
        RCLCPP_INFO(get_logger(), "读取当前位置...");
        for (int i = 0; i < 4; ++i) {
            hold_q_[i] = read_q(i);
            RCLCPP_INFO(get_logger(), " leg%d: [%.3f, %.3f, %.3f]",
                i, hold_q_[i][0], hold_q_[i][1], hold_q_[i][2]);
        }

        // 启动控制定时器
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int>(DT*1e6)),
            std::bind(&HoldNode::control_loop, this));

        RCLCPP_INFO(get_logger(), "保持节点就绪，锁定当前姿态");
    }

    ~HoldNode() {
        for (auto& [_, m] : motors_) m->unlock_motor();
    }

private:
    std::map<std::pair<int,int>, std::shared_ptr<MotorDriver>> motors_;
    std::array<std::array<float,3>,4> hold_q_{};
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<float,3> read_q(int leg) {
        std::array<float,3> q{};
        for (int j = 0; j < 3; ++j) {
            auto& m = motors_.at({leg,j});
            m->refresh_motor_status();
            q[j] = JSIGN[leg][j] * (m->get_motor_pos() - ZERO[leg][j]);
        }
        return q;
    }

    void control_loop() {
        for (int i = 0; i < 4; ++i) {
            auto q = hold_q_[i];
            for (int j = 0; j < 3; ++j) {
                auto& m = motors_.at({i,j});
                float raw = ZERO[i][j] + JSIGN[i][j] * q[j];
                m->motor_mit_cmd(raw, 0.f, KP, KD, 0.f);
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HoldNode>());
    rclcpp::shutdown();
    return 0;
}
