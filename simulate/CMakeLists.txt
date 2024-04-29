cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(mujoco REQUIRED)
include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

FILE (GLOB SIM_SRC
        src/mujoco/*.cc
        src/unitree_sdk2_bridge/*.cc)

set(SIM_DEPENDENCIES 
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp)


add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco  ${SIM_DEPENDENCIES})

add_executable(test test/test_unitree_sdk2.cpp)

SET(CMAKE_BUILD_TYPE Release)