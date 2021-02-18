cmake_minimum_required(VERSION 3.5)

project(swarm LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include_directories(mavlink/include/mavlink/v2.0)
include_directories(include)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(mavros_msgs REQUIRED)

find_package(std_msgs REQUIRED)
include_directories(${std_msgs_INCLUDE_DIRS})
find_package(geometry_msgs REQUIRED)
include_directories(${geometry_msgs_INCLUDE_DIRS})

set(SWARM_SRC src/main.cpp src/navigation.cpp
    src/guidance.cpp src/pid.cpp src/gnc.cpp)

add_executable(swarmcontrol ${SWARM_SRC})
target_link_libraries(swarmcontrol ${roscpp_LIBRARIES})
