cmake_minimum_required(VERSION 2.8.3)
project(eurobench_benchmark_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  madrob_msgs
  madrob_srvs
  beast_msgs
  beast_srvs
  sensor_msgs
  eurobench_bms_msgs_and_srvs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)