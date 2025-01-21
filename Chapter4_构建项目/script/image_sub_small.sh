#!/bin/bash

cd ~/ros2_ws/ros2CV_ws;
source /opt/ros/jazzy/setup.bash;
source install/local_setup.bash;
ros2 run topic2 image_sub;
cd ~/scripts;

