# Dog_VMC

四足机器人虚拟模型控制（VMC）项目。

## 新设备使用

### 1. 安装依赖

```bash
sudo apt-get update
sudo apt-get install build-essential cmake libspdlog-dev libfmt-dev \
    libeigen3-dev libboost-system-dev python3-dev pybind11-dev ccache \
    python3-numpy python3-matplotlib python3-pygame can-utils
```

### 2. 克隆项目

```bash
git clone https://github.com/HaoRan666666/Dog_VMC.git
cd Dog_VMC
```

### 3. 编译

```bash
# 编译 Controller（Python 绑定）
cd Controller
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON
make -j$(nproc)

# 编译 motors（Python 绑定）
cd ../../Module/motors
mkdir build && cd build
cmake ..
make -j$(nproc)

cd ../..
```

### 4. 配置 CAN 总线

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 up type can bitrate 1000000
```

### 5. 运行测试

```bash
# 实机模式（验证所有电机通信是否正常及方向）
python3 tests/test_fk_viz_quad_3d.py

# 仿真模式（不需要电机）
python3 tests/test_fk_viz_quad_3d.py --sim
```
