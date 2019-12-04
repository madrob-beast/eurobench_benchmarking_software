cmake_minimum_required(VERSION 2.8.3)
project(eurobench_rosbag_recorder)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    eurobench_bms_msgs_and_srvs
)

catkin_package(
)