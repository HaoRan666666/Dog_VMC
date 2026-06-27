# Dog_VMC

四足机器人 IK + MIT 位置控制项目。

## 1. 系统概述

| 特性 | 说明 |
|---|---|
| 轨迹规划 | 复合摆线（cycloid）足端轨迹 |
| 运动学 | 3-DOF 腿 FK/IK + 雅可比 Jᵀ |
| 步态 | Trot 对角小跑（duty=0.6） |
| 控制模式 | MIT（位置 + 刚度 + 阻尼 + 可选重力前馈） |
| 控制频率 | 200 Hz |
| 遥控 | Xbox 手柄 → ROS2 joy_node |

### 坐标系

| 坐标系 | X | Y | Z |
|---|---|---|---|
| 机身 | 前方 | 左方 | 上方 |

### 腿编号

| 编号 | 名称 |
|---|---|
| 0 | LF 左前 |
| 1 | RF 右前 |
| 2 | LB 左后 |
| 3 | RB 右后 |

---

## 2. 编译

### 依赖

```bash
sudo apt update
sudo apt install build-essential cmake libspdlog-dev libfmt-dev \
    pybind11-dev python3-dev python3-pyserial \
    python3-numpy python3-matplotlib can-utils \
    joystick ros-humble-joy
```

### 构建

```bash
cd Dog_VMC
source /opt/ros/humble/setup.bash

# motors（首次需单独编译）
cd Module/motors && mkdir -p build && cd build
cmake .. && make -j$(nproc)
cd ../../..

# colcon 构建全部 ROS2 包
colcon build
```

---

## 3. 上电准备

### 3.1 手柄

```bash
# 8BitDo Ultimate 2C: 关机 → 按住 Start+X → 插USB（X-input 模式）
ls /dev/input/js0        # 必须存在
jstest /dev/input/js0    # 验证

#香橙pi上要运行不然识别不到手柄
sudo modprobe xpad
sudo modprobe joydev
echo 2dc8 310a | sudo tee /sys/bus/usb/drivers/xpad/new_id

ros2 run joy joy_node    # 启动手柄节点
```

### 3.2 CAN 总线

```bash
for i in 0 1 2 3; do
    sudo ip link set can$i type can bitrate 1000000
    sudo ip link set up can$i
    sudo ip link set can$i txqueuelen 64
done
ip -details link show can0 | grep -E 'state|qlen'  # 验证
```


## 4. 启动

```bash
cd Dog_VMC
source /opt/ros/humble/setup.bash
source install/setup.bash

# 一键启动
ros2 launch quad_control quad.launch.py

# 或手动
ros2 run joy joy_node &
ros2 run quad_control quad_control &
```

---

## 5. 遥控操作

| 按键 | 功能 |
|---|---|
| RB | 使能 / 失能急停 |
| B | 站立 ↔ 趴下 |
| A | 站立 ↔ Trot（退出 Trot 需先松摇杆） |
| **X** | **重力前馈 ON/OFF**（默认关闭） |
| 左摇杆上下 | Trot: 前进/后退（步长 0~0.15m） |
| 左摇杆左右 | Trot: 横移（±0.06m） |
| 右摇杆左右 | Trot: 旋转（±0.04m） |
| RT/LT | 站立: 升高/降低机身 |
| 十字键上下 | 站立: 调抬腿高度 |

### 状态机

```
DISABLED ──RB──→ CROUCH ──B──→ STANDING ──A──→ TROTTING
   ↑                │            │              │
   └────RB──────────┴────RB──────┴──────RB──────┘
```

---

## 6. 辅助节点

| 节点 | 用法 | 说明 |
|---|---|---|
| `quad_control` | `ros2 run quad_control quad_control` | 遥控控制 |
| `hold_node` | `ros2 run quad_control hold_node` | 读取当前位置并锁住 |
| `read_node` | `ros2 run quad_control read_node` | 100Hz 打印电机状态，不发指令 |
| `imu_node` | `ros2 run imu_ros2 imu_node /dev/ttyACM0` | IMU 数据 |

---

## 7. CAN 诊断

### 查看状态

```bash
for i in 0 1 2 3; do
    echo "can$i: TX=$(cat /sys/class/net/can$i/statistics/tx_packets) rx=$(cat /sys/class/net/can$i/statistics/rx_packets) err=$(cat /sys/class/net/can$i/statistics/tx_errors)"
    ip -details link show can$i | grep -E 'can state|berr-counter'
done
```
### 报文间隔

```bash
candump -tz can0  
```


## 8. 参数速查

### 运动学

| 参数 | 值 |
|---|---|
| L1（大腿） | 0.2125 m |
| L2（小腿） | 0.25025 m |
| HIP_DX | 0.06 m |
| HIP_DY | 0.082 m |
| FOOT_X_OFF | {0,0,-0.03,-0.03}（后腿后移 3cm） |

### 高度

| 参数 | 值 |
|---|---|
| 趴下 | -0.15 m |
| 默认站立 | -0.25 m |
| 最高站立 | -0.38 m |

### Trot

| 参数 | 值 |
|---|---|
| 周期 | 0.8 s |
| 占空比 | 0.60 |
| 最大步长 | 0.15 m |
| 最大横移 | 0.06 m |
| 最大偏航 | 0.04 m |

### PD

| 参数 | 值 |
|---|---|
| KP | 100 |
| KD | 5 |

---

## 9. 测试脚本（Python）

```bash
python3 tests/test_stand.py --legs all            # 起立
python3 tests/test_quad_trot.py                   # Trot 步态
python3 tests/test_gravity_ff.py --leg 0          # 重力前馈单腿测试
python3 tests/test_fk_viz_quad.py                 # FK 可视化
```

---

## 10. 查看实时数据

```bash
ros2 topic echo /joy           # 手柄
ros2 topic echo /imu           # IMU
candump can0                   # CAN 帧
candump -e can0                # 含错误帧
cat /proc/net/can/stats        # CAN 负载统计
```
