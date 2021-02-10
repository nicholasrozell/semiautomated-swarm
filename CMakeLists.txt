cmake_minimum_required(VERSION 3.5)

project(mavcontrol LANGUAGES CXX)

find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(mavlink/include/mavlink/v2.0)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${std_msgs_INCLUDE_DIRS})

set(MAVCTRL_SRC mavlink_control.cpp autopilot_interface.cpp
    serial_port.cpp udp_port.cpp)
add_executable(mavlinkcontrol ${MAVCTRL_SRC})
target_link_libraries(mavlinkcontrol ${roscpp_LIBRARIES})

set(COMM_SRC comm_control.cpp comm_interface.cpp
    serial_port.cpp udp_port.cpp)
add_executable(commcontrol ${COMM_SRC})
target_link_libraries(commcontrol ${roscpp_LIBRARIES})

