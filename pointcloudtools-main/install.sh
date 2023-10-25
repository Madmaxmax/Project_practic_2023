#!/usr/bin/bash

sudo apt install -y liblz4-dev
pip install --extra-index-url https://rospypi.github.io/simple/ rosbag
pip install roslz4 --extra-index-url https://rospypi.github.io/simple
pip install --extra-index-url https://rospypi.github.io/simple/ ros-numpy
pip install py3rosmsgs
