cmake_minimum_required(VERSION 3.14)
project(quad_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(KIN_SRC
    ${CMAKE_SOURCE_DIR}/../../kinematics/src/types.cpp
    ${CMAKE_SOURCE_DIR}/../../kinematics/src/leg_kinematics.cpp
)
set(KIN_INC ${CMAKE_SOURCE_DIR}/../../kinematics/include)

set(MOTOR_DIR ${CMAKE_SOURCE_DIR}/../../Module/motors)
set(MOTOR_INC
    ${MOTOR_DIR}/include
    ${MOTOR_DIR}/src/drivers/lro
    ${MOTOR_DIR}/src/drivers/rs02
    ${MOTOR_DIR}/src/protocol
    ${MOTOR_DIR}/src
)
set(MOTOR_LIBS
    ${MOTOR_DIR}/build/libmotors.a
    ${MOTOR_DIR}/build/src/drivers/lro/liblro_motors.a
    ${MOTOR_DIR}/build/src/drivers/rs02/librs02_motors.a
    ${MOTOR_DIR}/build/src/protocol/can/libmotors_can.a
    ${MOTOR_DIR}/build/src/protocol/canfd/libmotors_canfd.a
)

add_executable(quad_control src/control_node.cpp ${KIN_SRC})
target_include_directories(quad_control PRIVATE ${KIN_INC} ${MOTOR_INC})
ament_target_dependencies(quad_control rclcpp sensor_msgs)
target_link_libraries(quad_control ${MOTOR_LIBS} spdlog fmt pthread)

install(TARGETS quad_control DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
