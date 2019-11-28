cmake_minimum_required(VERSION 2.8.3)
project(eurobench_bms_msgs_and_srvs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    std_srvs
    message_generation
)

add_message_files(
  FILES
  BenchmarkServerState.msg
)

add_service_files(
  FILES
  StartBenchmark.srv
  StopBenchmark.srv
  BenchmarkServerRobotNames.srv
  MadrobSettings.srv
  MadrobBenchmarkType.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_generation roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)