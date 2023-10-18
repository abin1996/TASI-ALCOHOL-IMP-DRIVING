#!/bin/bash

SUBJECT=$1
# NOW=$1
NOW=$(date +"%d-%m-%y_%H-%M-%S")
echo $NOW
echo $SUBJECT
TARGET_FOLDER=$HOME/DATA_COLLECTION/$SUBJECT/$NOW 

mkdir -p $TARGET_FOLDER/brac
python record_brac_can_messages.py --filename $TARGET_FOLDER/brac/brac_messages_$NOW.csv --db /home/iac_user/data_collection_scripts/dadss_breath_sensor_fixed.dbc