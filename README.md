# Dog_VMC

四足机器人 IK + MIT 位置控制项目。

## 新设备使用

### 1. 安装依赖

```bash
sudo apt update
sudo apt install build-essential cmake libspdlog-dev libfmt-dev \
    pybind11-dev python3-dev python3-pyserial \
    python3-numpy python3-matplotlib can-utils \
    joystick ros-humble-joy
```

### 2. 克隆项目

```bash
git clone https://github.com/HaoRan666666/Dog_VMC.git
cd Dog_VMC
```

### 3. 编译

```bash
# motors（首次需单独编译）
cd Module/motors && mkdir -p build && cd build
cmake .. && make -j$(nproc)
cd ../../..

# 全部 ROS2 包
source /opt/ros/humble/setup.bash
colcon build
```

### 4. 手柄配置（Orange Pi）

```bash
# 8BitDo Ultimate 2C: 关机 → 按住 Start+X → 开机/插USB（X-input 模式）
# 确认设备出现：
ls /dev/input/js0
jstest /dev/input/js0          # 推摇杆看数值变化

# 启动手柄驱动
ros2 run joy joy_node
# 另开终端验证
ros2 topic echo /joy
```

### 5. CAN 总线（Orange Pi，开机自启）

```bash
sudo tee /etc/systemd/system/can-setup.service << 'EOF'
[Unit]
Description=Setup CAN interfaces
After=multi-user.target
[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c 'for i in 0 1 2 3; do ip link set can$i type can bitrate 1000000; ip link set up can$i; done'
[Install]
WantedBy=multi-user.target
EOF
sudo systemctl enable --now can-setup.service
ip link show type can   # 验证全部 UP
```

### 6. 运行

```bash
# 一键启动
source install/setup.bash
ros2 launch quad_control quad.launch.py

# 或手动
ros2 run joy joy_node &
ros2 run quad_control quad_control &
```

### 操作

| 按键 | 功能 |
|---|---|
| RB | 使能 / 失能急停 |
| B | 站立 / 趴下 |
| A | 站立 / Trot（从 Trot 退出需先松摇杆） |
| 左摇杆 | 前进/后退 + 横移 |
| 右摇杆左右 | 旋转 |
| RT/LT | 调站立高度 |
| 十字键上下 | 调抬腿高度 |

详见 [docs/使用说明书.md](docs/使用说明书.md)
