#!/bin/bash

SUBJECT=$2
NOW=$1
SESSIONNAME=$3
echo $NOW
echo $SUBJECT
TARGET_FOLDER=$HOME/DATA_COLLECTION/$SUBJECT/$SESSIONNAME/$NOW 

mkdir -p $TARGET_FOLDER
cp -r $HOME/Default_Folders/DATA_COLLECTION/* $TARGET_FOLDER/

touch "$TARGET_FOLDER/event_signal/joystick.txt"

rostopic echo joy >> "$TARGET_FOLDER/event_signal/joystick.txt" &
rosbag record -q --split --duration 60 -o $TARGET_FOLDER/images1/images1.bag -b 0 /camera1/usb_cam1/image_raw/compressed &
rosbag record -q --split --duration 60 -o $TARGET_FOLDER/images2/images2.bag -b 0 /camera2/usb_cam2/image_raw/compressed &
rosbag record -q --split --duration 60 -o $TARGET_FOLDER/images3/images3.bag -b 0 /camera3/usb_cam3/image_raw/compressed &
rosbag record -q --split --duration 60 -o $TARGET_FOLDER/images4/images4.bag -b 0 /camera4/usb_cam4/image_raw/compressed &
rosbag record -q --split --duration 60 -o $TARGET_FOLDER/gps/gps.bag -b 0 /tcpfix /tcptime /tcpvel &
python3 /home/iac_user/data_collection_scripts/record_driving_can_messages.py --filename $TARGET_FOLDER/can/can_messages_$NOW.csv --db /home/iac_user/data_collection_scripts/446w_iupui_fixed.dbc


