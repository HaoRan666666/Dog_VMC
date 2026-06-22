cmake_minimum_required(VERSION 3.14)
project(kinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 REQUIRED)

add_library(kinematics_py MODULE
    src/types.cpp
    src/leg_kinematics.cpp
    src/pybind_module.cpp
)
target_include_directories(kinematics_py PRIVATE include)
target_link_libraries(kinematics_py PRIVATE pybind11::module)
set_target_properties(kinematics_py PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
