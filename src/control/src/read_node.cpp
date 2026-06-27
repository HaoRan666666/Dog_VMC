/**
 * 电机只读节点 — 仅读取位置/速度/电流/温度，不发任何指令
 *
 * 用途：调试/监控，不影响电机状态
 * 用法：ros2 run quad_control read_node
 */

#include <motor_driver.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <map>
#include <memory>
#include <string>
#include <tuple>

// ── 常量 ──
constexpr float DT = 0.01f;  // 打印频率 100Hz

// 电机映射：腿 → (ABD_ID, HIP_ID, KNEE_ID, CAN口)
const std::map<int, std::tuple<int,int,int,std::string>> MOTORS = {
    {0, {1,2,3,"can0"}}, {1, {1,2,3,"can1"}}, {2, {1,2,3,"can2"}}, {3, {1,2,3,"can3"}}
};

static const char* JOINT_NAMES[3] = {"ABD", "HIP", "KNEE"};
static const char* LEG_NAMES[4] = {"LF", "RF", "LB", "RB"};

class ReadNode : public rclcpp::Node {
public:
    ReadNode() : Node("read_node") {
        // 初始化 12 个电机（和 quad_control 一样的 init 流程）
        for (auto& [leg, ids] : MOTORS) {
            int a = std::get<0>(ids), h = std::get<1>(ids), k = std::get<2>(ids);
            auto& can = std::get<3>(ids);
            for (auto [j, id] : {std::pair{0,a}, {1,h}, {2,k}}) {
                auto m = MotorDriver::create_motor(id, "CAN", can, "LRO_CAN", 2);
                m->init_motor();                              // unlock → MIT → lock
                motors_[{leg,j}] = m;
            }
        }

        // 定时器驱动打印
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int>(DT*1e6)),
            std::bind(&ReadNode::print_all, this));

        RCLCPP_INFO(get_logger(), "只读节点就绪，%.0fHz 打印", 1.f/DT);
    }

    // 析构：全部电机失能
    ~ReadNode() {
        for (auto& [_, m] : motors_) m->unlock_motor();
    }

private:
    std::map<std::pair<int,int>, std::shared_ptr<MotorDriver>> motors_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 每帧打印全部电机状态
    void print_all() {
        printf("\n══════ %s ══════\n",
            std::to_string(this->now().seconds()).c_str());

        for (int i = 0; i < 4; ++i) {
            printf("%s:", LEG_NAMES[i]);                      // LF/RF/LB/RB
            for (int j = 0; j < 3; ++j) {
                auto& m = motors_.at({i,j});

                // refresh_motor_status 发零 MIT 触发电机回包，更新内部数据
                m->refresh_motor_status();

                // 读四个物理量：位置(rad) 速度(rad/s) 电流/扭矩(Nm) 温度(℃)
                printf(" %s=%.2f/%.1f/%.1f/%.0fC",
                    JOINT_NAMES[j],                           // ABD/HIP/KNEE
                    m->get_motor_pos(),                       // 原始电机位置 (rad)
                    m->get_motor_spd(),                       // 速度 (rad/s)
                    m->get_motor_current(),                   // 电流/扭矩 (Nm)
                    m->get_motor_temperature());              // 线圈温度 (℃)
            }
            printf("\n");
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadNode>());
    rclcpp::shutdown();
    return 0;
}
