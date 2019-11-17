cmake_minimum_required(VERSION 2.8.3)
project(eurobench_benchmark_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  madrob_msgs
  madrob_srvs
  sensor_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BenchmarkServerState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StartBenchmark.srv
  StopBenchmark.srv
  BenchmarkServerSettings.srv
  MadrobSettings.srv
  MadrobBenchmarkType.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  madrob_msgs
  madrob_srvs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############