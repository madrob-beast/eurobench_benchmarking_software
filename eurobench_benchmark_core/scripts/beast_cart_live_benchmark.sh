#!/bin/bash

export ROS_IP='192.168.1.100'  # The IP of the GUI PC
export ROS_MASTER_URI='http://192.168.1.3:11311'  # The IP of the BEAST onboard PC
roslaunch eurobench_benchmark_core beast_cart_gui.launch
