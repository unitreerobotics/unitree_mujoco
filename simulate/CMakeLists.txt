cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

file(GLOB SIM_SRC
    src/joystick/joystick.cc
    src/mujoco/*.cc
)

set(SIM_DEPENDENCIES
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp
    unitree_sdk2
    boost_program_options
    fmt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco ${SIM_DEPENDENCIES})

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)