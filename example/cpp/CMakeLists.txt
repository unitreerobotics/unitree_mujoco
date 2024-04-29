cmake_minimum_required(VERSION 3.16)
project(stand_go2)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

add_executable(stand_go2 stand_go2.cpp)

SET(CMAKE_BUILD_TYPE Release)