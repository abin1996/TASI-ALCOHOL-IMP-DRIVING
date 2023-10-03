#!/bin/bash


NOW=$(date +"%d-%m-%y_%H:%M:%S")
echo $NOW

mkdir -p $HOME/DATA_COLLECTION/$NOW
cp -r $HOME/Default_Folders/DATA_COLLECTION/* $HOME/DATA_COLLECTION/$NOW/

touch "$HOME/DATA_COLLECTION/$NOW/joy/joystick.txt"

rostopic echo joy >> "$HOME/DATA_COLLECTION/$NOW/joy/joystick.txt" &
rosbag record --split --duration 60 -o $HOME/DATA_COLLECTION/$NOW/images1/images1.bag -b 0 /camera1/usb_cam1/image_raw/compressed &
rosbag record --split --duration 60 -o $HOME/DATA_COLLECTION/$NOW/images2/images2.bag -b 0 /camera2/usb_cam2/image_raw/compressed &
rosbag record --split --duration 60 -o $HOME/DATA_COLLECTION/$NOW/images3/images3.bag -b 0 /camera3/usb_cam3/image_raw/compressed &
rosbag record --split --duration 60 -o $HOME/DATA_COLLECTION/$NOW/images4/images4.bag -b 0 /camera4/usb_cam4/image_raw/compressed &
rosbag record --split --duration 60 -o $HOME/DATA_COLLECTION/$NOW/gps/gps.bag -b 0 /tcpfix /tcptime /tcpvel

