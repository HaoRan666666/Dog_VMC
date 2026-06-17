cmake_minimum_required(VERSION 3.16)
project(dog_vmc VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Sub-projects ───
# Existing motor driver module
add_subdirectory(Module/motors)

# VMC controller (new)
add_subdirectory(Controller)
