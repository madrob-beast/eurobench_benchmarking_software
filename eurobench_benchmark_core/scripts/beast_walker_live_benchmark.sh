#!/bin/bash

export ROS_IP='10.0.0.177'  # The IP of the GUI PC
export ROS_MASTER_URI='http://10.0.0.128:11311'  # The IP of the BEAST onboard PC
roslaunch eurobench_benchmark_core beast_walker_gui.launch
