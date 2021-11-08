#!/bin/bash

cd ~/catkin_ws/src/

git clone https://github.com/madrob-beast/eurobench_benchmarking_software.git
git clone https://github.com/madrob-beast/beast.git
git clone https://github.com/madrob-beast/beast_srvs.git
git clone https://github.com/madrob-beast/beast_msgs.git
git clone https://github.com/madrob-beast/madrob_beast_pi.git

sudo apt update
sudo apt install -y python-pandas

# install latest version of PyYAML and prevent the error:
#    AttributeError: 'module' object has no attribute 'FullLoader'
sudo apt install -y python-pip
pip install -U PyYAML

