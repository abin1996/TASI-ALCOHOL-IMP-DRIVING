#!/bin/bash

SUBJECT=$2
NOW=$1
echo $NOW
echo $SUBJECT
TARGET_FOLDER=$HOME/DATA_COLLECTION/$SUBJECT/$NOW 

mkdir -p $TARGET_FOLDER/can

python record_driving_can_messages.py --filename $TARGET_FOLDER/can/can_messages_$NOW.csv --db /home/iac_user/data_collection_scripts/446w_iupui_fixed.dbc