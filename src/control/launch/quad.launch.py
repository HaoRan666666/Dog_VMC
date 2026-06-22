"""
四足机器人遥控控制 — 一键启动所有节点

用法:
  ros2 launch control quad.launch.py
  ros2 launch control quad.launch.py imu_device:=/dev/ttyUSB0
  ros2 launch control quad.launch.py imu_enable:=false  # 不启动 IMU
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    imu_enable = LaunchConfiguration("imu_enable").perform(context)
    imu_device = LaunchConfiguration("imu_device").perform(context)

    nodes = [
        # 手柄驱动
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),
        # 主控制器
        Node(
            package="quad_control",
            executable="quad_control",
            name="quad_control",
            output="screen",
        ),
    ]

    # IMU（可选）
    if imu_enable.lower() == "true":
        nodes.append(
            Node(
                package="imu_ros2",
                executable="imu_node",
                name="imu_node",
                output="screen",
                arguments=[imu_device],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("imu_enable", default_value="true",
                              description="是否启动 IMU 节点"),
        DeclareLaunchArgument("imu_device", default_value="/dev/ttyACM0",
                              description="DM-IMU 串口设备路径"),
        OpaqueFunction(function=launch_setup),
    ])
