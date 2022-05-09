#!/bin/bash

export ROS_IP='192.168.0.10'  # The IP of the GUI PC
#export ROS_HOSTNAME='beast-madrob-gui'  # The hostname of the GUI PC
export ROS_MASTER_URI='http://192.168.0.2:11311'  # The ROS master URI of the MADROB onboard PC
roslaunch eurobench_benchmark_core madrob_gui.launch
