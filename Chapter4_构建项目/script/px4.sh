#!/bin/bash
gnome-terminal -t "Micro-XRCE-DDS-Agent" -x bash -c "sudo ldconfig /usr/local/lib/;
MicroXRCEAgent udp4 -p 8888;"
sleep 3s
gnome-terminal -t "PX4-Autopilot" -x bash -c "cd PX4-Autopilot/;make px4_sitl gz_x500;"
sleep 7s
gnome-terminal -t "ros2" -x bash -c "cd ~/ws_sensor_combined/;
source /opt/ros/humble/setup.bash;
source install/local_setup.bash;
#ros2 run vehicle_gps_position_listener;"
sleep 10s

#ros2 run px4_ros_com offboard_control_1

#ros2 run px4_ros_com vehicle_gps_position_listener 

#ros2 launch px4_ros_com sensor_combined_listener.launch.py

