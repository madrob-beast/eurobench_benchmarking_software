#!/bin/bash

export ROS_IP='10.0.0.176'  # The IP of the GUI PC
export ROS_MASTER_URI='http://10.0.0.128:11311'  # The IP of the BEAST onboard PC
roslaunch eurobench_benchmark_core beast_gui.launch
