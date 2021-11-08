#!/bin/bash

cd ~/catkin_ws/src/

git clone https://github.com/madrob-beast/eurobench_benchmarking_software.git
git clone https://github.com/madrob-beast/beast_scan_filter.git
git clone https://github.com/madrob-beast/beast_odometry_publisher.git
git clone https://github.com/madrob-beast/beast_localization.git
git clone https://github.com/madrob-beast/madrob.git
git clone https://github.com/madrob-beast/madrob_srvs.git
git clone https://github.com/madrob-beast/madrob_msgs.git
git clone https://github.com/madrob-beast/madrob_beast_pi.git

sudo apt update
sudo apt install -y htop python-pandas ros-melodic-rqt-gui ros-melodic-rqt-gui-cpp ros-melodic-amcl ros-melodic-tf2 ros-melodic-tf ros-melodic-map-server

# install latest version of PyYAML and prevent the error:
#    AttributeError: 'module' object has no attribute 'FullLoader'
sudo apt install -y python-pip
pip install -U PyYAML

