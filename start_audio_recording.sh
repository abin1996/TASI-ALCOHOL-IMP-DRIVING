#!/bin/bash
NOW=$1
SUBJECT=$2
echo $NOW
TARGET_FOLDER=$HOME/DATA_COLLECTION/$SUBJECT/$NOW 
mkdir -p $TARGET_FOLDER/audio
roslaunch audio_capture capture_to_file.launch dst:=$TARGET_FOLDER/audio/voice.mp3
