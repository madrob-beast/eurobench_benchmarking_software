cmake_minimum_required(VERSION 2.8.3)

project(benchmark_gui)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp
  std_msgs
  std_srvs
  sensor_msgs
  eurobench_bms_msgs_and_srvs
)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/benchmark_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
