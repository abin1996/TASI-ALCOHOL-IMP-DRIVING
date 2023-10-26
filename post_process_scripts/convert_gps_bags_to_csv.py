import os
import rosbag
import sys
import csv
import time
import string
import shutil

# RECORDING_FOLDER_NAME = '11-08-23_15:24:04'

# SOURCE_CAMERA_BAG_FOLDER = "/home/iac_user/DATA_COLLECTION/" + RECORDING_FOLDER_NAME
# SAVE_FOLDER_FOR_CAMERA_IMAGES = '/home/iac_user/PROCESSED_DATA_COLLECTION/' + RECORDING_FOLDER_NAME
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

def extract_gps(input_folder,output_folder):


    listOfBagFiles = [f for f in os.listdir(input_folder) if f[-4:] == ".bag"]  # get list of only bag files in current dir.
    numberOfFiles = str(len(listOfBagFiles))
    print("reading all " + numberOfFiles + " bagfiles in current directory: \n")
    count = 0
    for bagFile in sorted(listOfBagFiles):
        count += 1
        print("reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile)
        # access bag
        bag = rosbag.Bag(input_folder+"/" + bagFile)
        bagContents = bag.read_messages()
        bagName = bag.filename

        # create a new directory
        folder = output_folder + "/gps/" + bagFile.rstrip(".bag")
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



if __name__ == '__main__': 

    start_time = time.time()

    # sub_folder_name = 'Driving backward'
    sub_folder_name = 'Driving forward'
    # sub_folder_name = 'Eye tracking'
    # sub_folder_name = 'Parking'
    
    SUB_FOLDER_NAME = sub_folder_name + '/'

    SUB_SUB_FOLDER_NAME = sub_folder_name +'_'

    SOURCE_FOLDER = '/home/iac_user/DATA_COLLECTION/SubjectAnn/70-Alcohol/24-10-23_13-49-28/gps'

    SOURCE_GPS_BAG_FOLDER = SOURCE_FOLDER +'/' + sub_folder_name

    num_sub_sub_folder = len([folder for folder in os.listdir(SOURCE_GPS_BAG_FOLDER) if os.path.isdir(os.path.join(SOURCE_GPS_BAG_FOLDER, folder))])

    for i in range(num_sub_sub_folder):
            SUB_SUB_FOLDER_IND = str(i+1)

            SAVE_FOLDER_FOR_GPS = '/home/iac_user/POST_PROCESS/SubjectAnn/70-Alcohol/' + SUB_FOLDER_NAME + SUB_SUB_FOLDER_NAME + SUB_SUB_FOLDER_IND

            gps_input_folder = SOURCE_GPS_BAG_FOLDER + "/" + SUB_SUB_FOLDER_NAME + SUB_SUB_FOLDER_IND
            
            extract_gps(gps_input_folder,SAVE_FOLDER_FOR_GPS)


    print("GPS files generated")

    end_time = time.time()
    print("Time elapsed: ", end_time - start_time, " seconds")
