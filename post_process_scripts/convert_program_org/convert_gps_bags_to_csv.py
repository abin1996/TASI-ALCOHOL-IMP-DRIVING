import os
import rosbag
import sys
import csv
import time
import string
import shutil

RECORDING_FOLDER_NAME = '11-08-23_15:24:04'

SOURCE_CAMERA_BAG_FOLDER = "/home/iac_user/DATA_COLLECTION/" + RECORDING_FOLDER_NAME
SAVE_FOLDER_FOR_CAMERA_IMAGES = '/home/iac_user/PROCESSED_DATA_COLLECTION/' + RECORDING_FOLDER_NAME
# verify correct input arguments: 1 or 2
# if len(sys.argv) > 2:
#     print("invalid number of arguments: " + str(len(sys.argv)))
#     print("should be 2: 'bag2csv.py' and 'bagName'")
#     print("or just 1  : 'bag2csv.py'")
#     sys.exit(1)
# elif len(sys.argv) == 2:
#     listOfBagFiles = [sys.argv[1]]
#     numberOfFiles = "1"
#     print("reading only 1 bagfile: " + str(listOfBagFiles[0]))
# elif len(sys.argv) == 1:
#     listOfBagFiles = [f for f in os.listdir(".") if f[-4:] == ".bag"]  # get list of only bag files in current dir.
#     numberOfFiles = str(len(listOfBagFiles))
#     print("reading all " + numberOfFiles + " bagfiles in current directory: \n")
#     for f in sorted(listOfBagFiles):
#         print(f)
#     print("\n press ctrl+c in the next 10 seconds to cancel \n")
#     time.sleep(10)
# else:
#     print("bad argument(s): " + str(sys.argv))  # shouldn't really come up
#     sys.exit(1)

listOfBagFiles = [f for f in os.listdir(SOURCE_CAMERA_BAG_FOLDER+"/gps") if f[-4:] == ".bag"]  # get list of only bag files in current dir.
numberOfFiles = str(len(listOfBagFiles))
print("reading all " + numberOfFiles + " bagfiles in current directory: \n")
count = 0
for bagFile in sorted(listOfBagFiles):
    count += 1
    print("reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile)
    # access bag
    bag = rosbag.Bag(SOURCE_CAMERA_BAG_FOLDER+"/gps/" + bagFile)
    bagContents = bag.read_messages()
    bagName = bag.filename

    # create a new directory
    folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/gps/" + bagFile.rstrip(".bag")
    print("bag nameL ",bagName)
    print("Target folder: ",folder)
    try:  # else already exists
        os.makedirs(folder,exist_ok=True)
    except:
        pass
    shutil.copyfile(bagName, os.path.join(folder, bagFile))

    # get list of topics from the bag
    listOfTopics = []
    for topic, msg, t in bagContents:
        if topic not in listOfTopics:
            listOfTopics.append(topic)

    for topicName in listOfTopics:
        # Create a new CSV file for each topic
        filename = os.path.join(folder, topicName.replace('/', '_slash_') + '.csv')
        with open(filename, 'w+', newline='') as csvfile:
            filewriter = csv.writer(csvfile, delimiter=',')
            firstIteration = True  # allows header row
            for subtopic, msg, t in bag.read_messages(topicName):
                # parse data from this instant
                msgString = str(msg)
                msgList = msgString.split('\n')
                instantaneousListOfData = []
                for nameValuePair in msgList:
                    splitPair = nameValuePair.split(':')
                    for i in range(len(splitPair)):  # should be 0 to 1
                        splitPair[i] = splitPair[i].strip()
                    instantaneousListOfData.append(splitPair)
                if firstIteration:  # header
                    headers = ["rosbagTimestamp"]  # first column header
                    for pair in instantaneousListOfData:
                        headers.append(pair[0])
                    filewriter.writerow(headers)
                    firstIteration = False
                values = [str(t)]  # first column will have rosbag timestamp
                for pair in instantaneousListOfData:
                    if len(pair) > 1:
                        values.append(pair[1])
                filewriter.writerow(values)
    bag.close()

print("Done reading all " + numberOfFiles + " bag files.")
