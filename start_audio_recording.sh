#!/bin/bash
NOW=$1
SUBJECT=$2
SESSIONNAME=$3
FILENAMETIME=$(date +"%Y-%m-%d_%H-%M-%S")
AUDIODEVICE=$(arecord --list-devices | grep H650e | grep -o 'card \([0-9]\+\): .* device \([0-9]\+\):' | sed -E 's/card ([0-9]+):.*device ([0-9]+):/hw:\1,\2/')
echo $NOW
TARGET_FOLDER=$HOME/DATA_COLLECTION/$SUBJECT/$SESSIONNAME/$NOW 
mkdir -p $TARGET_FOLDER/audio
roslaunch audio_capture capture_to_file.launch dst:=$TARGET_FOLDER/audio/voice_$FILENAMETIME.mp3 device:=$AUDIODEVICE
