SUBJECT=$1
cd $HOME/DATA_COLLECTION/$SUBJECT
LATEST_REC=$(echo $(ls -Art | tail -1))
echo ************************GPS*************************
cd $HOME/DATA_COLLECTION/$SUBJECT/$LATEST_REC/gps
rosbag info $(echo $(ls -t | grep .bag | sort -n |tail -2|head -1))

echo **********************CAMERA-1************************
cd $HOME/DATA_COLLECTION/$SUBJECT/$LATEST_REC/images1/
rosbag info $(echo $(ls -t | grep .bag | sort -n |tail -2|head -1))

echo **********************CAMERA-2************************
cd $HOME/DATA_COLLECTION/$SUBJECT/$LATEST_REC/images2/
rosbag info $(echo $(ls -t | grep .bag | sort -n |tail -2|head -1))

echo **********************CAMERA-3************************
cd $HOME/DATA_COLLECTION/$SUBJECT/$LATEST_REC/images3/
rosbag info $(echo $(ls -t | grep .bag | sort -n |tail -2|head -1))

echo **********************CAMERA-4************************
cd $HOME/DATA_COLLECTION/$SUBJECT/$LATEST_REC/images4/
rosbag info $(echo $(ls -t | grep .bag | sort -n |tail -2|head -1))

echo **********************CAN-MESSAGES************************
cd $HOME/DATA_COLLECTION/$SUBJECT/$LATEST_REC/can/
stat -c "%s" $(echo $(ls -t | grep .csv ))
