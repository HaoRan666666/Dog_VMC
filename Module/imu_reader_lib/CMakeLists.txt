cmake_minimum_required(VERSION 3.14)
project(IMUReader LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(imu_reader_lib STATIC
    src/crc16.cpp
    src/serial_port.cpp
    src/imu_parser.cpp
    src/imu_reader.cpp
)

target_include_directories(imu_reader_lib PUBLIC src)
target_compile_options(imu_reader_lib PRIVATE -Wall -Wextra -Wpedantic)

# Example executable
add_executable(imu_reader_example src/main.cpp)
target_link_libraries(imu_reader_example imu_reader_lib)
