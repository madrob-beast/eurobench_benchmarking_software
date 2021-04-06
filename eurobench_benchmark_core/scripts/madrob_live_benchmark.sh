#!/bin/bash

#export ROS_IP='TODO'  # The IP of the GUI PC
export ROS_HOSTNAME='TODO'  # The hostname of the GUI PC
export ROS_MASTER_URI='http://madrob:11311'  # The ROS master URI of the BEAST onboard PC
roslaunch eurobench_benchmark_core madrob_gui.launch
