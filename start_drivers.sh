#!/bin/bash
roslaunch usb_cam usb_test.launch &
P2=$!
rosrun reach_ros_node nmea_tcp_driver _host:=192.168.1.6 _port=12346 > /dev/null 2>&1 &
P3=$!
rosrun joy joy_node &
wait $P2 $P3
