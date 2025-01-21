#!/bin/bash

#gnome-terminal -t "image_sub_small"  --window --geometry=85x25+830+550 -- bash -c "
source /opt/ros/jazzy/setup.bash;
cd ~/ros2_ws/ros2CV_ws;
source install/setup.bash;
ros2 run topic2 image_pub;
exec bash;"

