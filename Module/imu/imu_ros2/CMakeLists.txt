cmake_minimum_required(VERSION 3.14)
project(imu_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
    msg/IMUData.msg
)

# IMU 串口库
add_library(imu_reader_lib STATIC
    src/crc16.cpp
    src/imu_parser.cpp
    src/imu_reader.cpp
    src/serial_port.cpp
)
target_include_directories(imu_reader_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/imu_ros2>
    $<INSTALL_INTERFACE:include/imu_ros2>
)

# IMU 节点
add_executable(imu_node src/imu_node.cpp)
target_include_directories(imu_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/imu_ros2>
)
ament_target_dependencies(imu_node rclcpp)
rosidl_target_interfaces(imu_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(imu_node imu_reader_lib)

install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})

ament_package()
