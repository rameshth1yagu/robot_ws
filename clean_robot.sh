#!/bin/bash
killall -9 ruby ign gazebo gz rviz2
sudo rm /dev/shm/fastrtps*
echo "Jetson ROS 2 memory cleared."