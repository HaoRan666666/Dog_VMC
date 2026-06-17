cmake_minimum_required(VERSION 3.16)
project(vmc_controller VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ─── Find dependencies ───
find_package(spdlog REQUIRED)

# Optional: Python bindings
option(BUILD_PYTHON_BINDINGS "Build pybind11 Python module" ON)
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED)
endif()

# ─── Library sources ───
set(LIBRARY_SOURCES
    src/types.cpp
    src/trajectory_planner.cpp
    src/leg_kinematics.cpp
    src/gait_generator.cpp
    src/state_estimator.cpp
    src/balance_controller.cpp
    src/quadruped_controller.cpp
)

set(LIBRARY_HEADERS
    include/vmc_controller/types.h
    include/vmc_controller/trajectory_planner.h
    include/vmc_controller/leg_kinematics.h
    include/vmc_controller/gait_generator.h
    include/vmc_controller/state_estimator.h
    include/vmc_controller/balance_controller.h
    include/vmc_controller/quadruped_controller.h
)

# ─── Static library ───
add_library(vmc_controller_lib STATIC ${LIBRARY_SOURCES})
target_include_directories(vmc_controller_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(vmc_controller_lib PUBLIC spdlog::spdlog)

# ─── Install ───
include(GNUInstallDirs)
install(TARGETS vmc_controller_lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES ${LIBRARY_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/vmc_controller
)

# ─── Python bindings ───
if(BUILD_PYTHON_BINDINGS AND pybind11_FOUND)
    pybind11_add_module(vmc_controller_py src/pybind_module.cpp)
    target_link_libraries(vmc_controller_py PRIVATE vmc_controller_lib)
    target_include_directories(vmc_controller_py PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    install(TARGETS vmc_controller_py
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

# ─── Optional: standalone test executable ───
option(BUILD_TESTS "Build test executables" OFF)
if(BUILD_TESTS)
    add_executable(test_core tests/test_core.cpp)
    target_link_libraries(test_core PRIVATE vmc_controller_lib)
endif()
