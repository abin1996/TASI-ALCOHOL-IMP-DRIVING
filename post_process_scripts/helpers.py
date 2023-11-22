
import os, os.path, shutil
from datetime import datetime,timedelta
import csv
import rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
import pytz
import pandas as pd
import math
import re
import logging

VIDEO_FRAMERATE = 30
MJPEG_VIDEO = 1
RAWIMAGE_VIDEO = 2
VIDEO_CONVERTER_TO_USE = "ffmpeg"

log = logging.getLogger(__name__)
def timestamp_to_date(timestamp):
    # convert the timestamp to a datetime object in the local timezone
    dt_object = datetime.fromtimestamp(timestamp)
    time = dt_object.time()
    return dt_object

def event_timestamp(filename):
    # opens the file, reads and stores each line into a list
    with open(filename) as file:
        lines = file.readlines()

    index_button_list=[]
    for i in range(7,len(lines),9): # get the line number of button in each sequence
        index_button_list.append(i)

    index_nonzero_button_list = [] # the list contains all button log that is of interest
    for m in index_button_list:
        if lines[m] == "buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n" \
                or lines[m] == "buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n" \
                or lines[m] == "buttons: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]\n" \
                or lines[m] == "buttons: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]\n":
            index_nonzero_button_list.append(m)

    target_line_index_list = []
    for p in index_nonzero_button_list:
        del_start = p-7 # the starting line number of zero button sequence
        del_end = p+1 # the ending line number of zero button sequence
        for i in range(del_start,del_end+1,1):
            target_line_index_list.append(i)

    # Write file
    # with open(event_timestamp_filename, "w+") as file:
        # iterate each line
    new_lines = []
    for number, line in enumerate(lines):
        # delete lines from the saved list
        if number in target_line_index_list:
            # file.write(line)
            new_lines.append(line)

    # # read file
    # with open(event_timestamp_filename, 'r') as fp:
    #     # read an store all lines into list
    #     new_lines = fp.readlines()

    new_index_button_list = []
    for i in range(7, len(new_lines), 9):  # get the line number of button in each sequence
        new_index_button_list.append(i)
    # print('button_list',index_button_list)

    event_A_timestamp_list = []  # the list contains all button log that represents event A
    event_B_timestamp_list = []  # the list contains all button log that represents event B
    event_X_timestamp_list = []  # the list contains all button log that represents event X
    event_Y_timestamp_list = []  # the list contains all button log that represents event Y


    for m in new_index_button_list:
        n = m - 4  # n is the line that contains the timestamp info
        timestamp = int(new_lines[n].split(':')[1])
        hr_min_sec = timestamp_to_date(timestamp)

        if new_lines[m] == "buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n":
            event_A_timestamp_list.append(hr_min_sec)
            # event_A_timestamp_list.append(new_lines[n])

        elif new_lines[m] == "buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n":
            event_B_timestamp_list.append(hr_min_sec)
            # event_A_timestamp_list.append(new_lines[n])

        elif new_lines[m] == "buttons: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]\n":
            event_X_timestamp_list.append(hr_min_sec)
            # event_X_timestamp_list.append(new_lines[n])

        elif new_lines[m] == "buttons: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]\n":
            event_Y_timestamp_list.append(hr_min_sec)
            # event_Y_timestamp_list.append(new_lines[n])

    all_events_dict = {'Eye tracking':event_A_timestamp_list,'Driving backward':event_B_timestamp_list,'Parking':event_X_timestamp_list,'Driving forward':event_Y_timestamp_list}
    return all_events_dict

def copy_files_to_subfolders(source_folder, subfolder_names):
    # Ensure the source folder exists
    if not os.path.exists(source_folder):
        print("The source folder '{}' does not exist.".format(source_folder))
        return

    # Create subfolders if they don't already exist
    for subfolder_name in subfolder_names:
        subfolder_path = os.path.join(source_folder, subfolder_name)
        os.makedirs(subfolder_path, exist_ok=True)

    # List all files in the source folder
    files = [f for f in os.listdir(source_folder) if os.path.isfile(os.path.join(source_folder, f))]

    # Copy each file to all subfolders
    for file in files:
        source_file_path = os.path.join(source_folder, file)
        for subfolder_name in subfolder_names:
            subfolder_path = os.path.join(source_folder, subfolder_name)
            destination_file_path = os.path.join(subfolder_path, file)
            shutil.copy2(source_file_path, destination_file_path)

def insert_missing_numbers(lst):
    if not lst:
        return []
    # Find the minimum and maximum values in the list
    min_num = min(lst)
    max_num = max(lst)
    # Generate a new list with missing sequential numbers
    complete_list = list(range(min_num, max_num + 1))
    return complete_list

def filtered_event_timestamp(timestamp_list):
    # Initialize the list for the filtered timestamps
    filtered_timestamps = []
    # Iterate through the timestamps and keep the first timestamp with a gap of more than 2 seconds
    for i in range(len(timestamp_list) - 1):
        current_time = timestamp_list[i]
        next_time = timestamp_list[i + 1]
        time_difference = (next_time - current_time).total_seconds()
        if time_difference <= 2:
            filtered_timestamps.append(current_time)

    filtered_timestamps = [timestamp for j, timestamp in enumerate(filtered_timestamps) if j == 0 or (timestamp - filtered_timestamps[j-1]).total_seconds() > 1]
    return filtered_timestamps

def extract_can_for_sub_category(source_file,save_file_path, timestamp_ranges):
    data = pd.read_csv(source_file)
    data['time'] = pd.to_datetime(data['time'], format='%d-%m-%y_%H:%M:%S.%f')
# Iterate through each timestamp range and create separate CSV files

    for i in range(len(timestamp_ranges)):
        EVENT_IND = i+1
        if len(timestamp_ranges[i]) == 2:
            start,end = timestamp_ranges[i]
            start_timestamp = pd.to_datetime(start)
            end_timestamp = pd.to_datetime(end)

            # Filter the data within the timestamp range
            filtered_data = data[(data["time"] >= start_timestamp) & (data["time"] <= end_timestamp)]
            # print(filtered_data)

            # Define the user-defined directory and file name
            user_defined_directory = save_file_path + "_" + str(EVENT_IND) + "/can/"  # Replace with your directory

            output_file_name = "can_data.csv"

            # Define the complete file path
            output_file_path = os.path.join(user_defined_directory, output_file_name)
            
            if os.path.exists(user_defined_directory):
                for file in os.listdir(user_defined_directory):
                    file_path = os.path.join(user_defined_directory, file)
                    os.remove(file_path) 
            if not os.path.exists(user_defined_directory):
                os.makedirs(user_defined_directory, exist_ok=True)  # Create the directory if it doesn't exist

            # # Create a new CSV file for the filtered data
            filtered_data.to_csv(output_file_path, index=False)
            print("CAN CSV file have been created at: ",output_file_path)
        else:
            print('Session',str(EVENT_IND),'Missing one controller button input')

def file_recategory(timestamp_list, org_folder_name, new_folder_name):
    event_timestamp_list = timestamp_list
    folder_path = org_folder_name + '/'+ new_folder_name
    filtered_timestamps = filtered_event_timestamp(event_timestamp_list)
    files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    files.sort()
    bag_time_list=[]

    for file in files:
        bag_sec_string = file.split('-')[5]

        bag_ind_string = bag_sec_string.split('_')[1]

        bag_ind = bag_ind_string.split('.')[0]

        datetime_str = file.split("_")[1]

        datetime_obj = datetime.strptime(datetime_str,"%Y-%m-%d-%H-%M-%S")

        bag_time_list.append((datetime_obj,int(bag_ind)))
    bag_time_list.sort(key=lambda a: a[1])
    event_bag_ind_list=[]

    for index, timestamp in enumerate(filtered_timestamps):
        for i in range(0,len(bag_time_list)):
            if i < len(bag_time_list)-1:
                start_time = bag_time_list[i][0]
                end_time = bag_time_list[i+1][0]
                # Check if the timestamp is within the time slot
                if start_time <= timestamp < end_time:
                    event_bag_ind_list.append(int(bag_time_list[i][1]))
                    break
            else:
                start_time = bag_time_list[i][0]
                end_time = start_time + timedelta(minutes=1)
                # Check if the timestamp is within the time slot
                if start_time <= timestamp < end_time:
                    event_bag_ind_list.append(int(bag_time_list[i][1]))
                    break

    # Initialize an empty list to store the sublists
    event_bag_ind_list_paired = []

    # Iterate through the list in steps of 2 [start_bag_ind, end_bag_ind]
    for i in range(0, len(event_bag_ind_list), 2):
        sublist = event_bag_ind_list[i:i + 2]  # Get two elements from the current position
        event_bag_ind_list_paired.append(sublist)  # Append the sublist to the result list

    complete_event_bag_ind_list = []
    for j in range(len(event_bag_ind_list_paired)):
        lst = event_bag_ind_list_paired[j]
        complete_lst = insert_missing_numbers(lst)
        complete_event_bag_ind_list.append(complete_lst)

    log.debug('Complete bag index: {}'.format(complete_event_bag_ind_list))
    # create the csv file for each session
    event_timestamp_paired = [] # Initialize the list for paried timestamps. The list would have the format:[[event1_start_timestamp,event1_end_timestamp],[e2_start,e2_end],...]

    for k in range(0, len(filtered_timestamps), 2):
        sub_lst = filtered_timestamps[k:k + 2]  # Get two elements from the current position
        event_timestamp_paired.append(sub_lst)  # Append the sublist to the result list

    for q in range(0,len(complete_event_bag_ind_list)):
        bag_start_time = bag_time_list[complete_event_bag_ind_list[q][0]][0]

        if len(event_timestamp_paired[q]) == 2:
            event_begin_time, event_finish_time = event_timestamp_paired[q]

            # Calculate the time differences
            video_start_time = event_begin_time - bag_start_time
            video_stop_time = event_finish_time - bag_start_time

            # Format the time differences as strings
            video_start_time_str = str(video_start_time)
            video_stop_time_str = str(video_stop_time)
            log.debug('Bag start time: {}'.format(bag_start_time))
            log.debug('Event start time:{}'.format(event_begin_time))
            log.debug('Event end time:{}'.format(event_finish_time))
            log.debug('Video start time:{}'.format(video_start_time_str))
            log.debug('Video end time:{}'.format(video_stop_time_str))

            subtest_ind = q + 1
            subfolder_name = new_folder_name + '_'+ str(subtest_ind)

            event_new_path = os.path.join(folder_path, subfolder_name)

            if not os.path.exists(event_new_path):
                os.makedirs(event_new_path)

                for p in range(len(complete_event_bag_ind_list[q])):
                    # print("P: ",p)
                    old_A_file_path = os.path.join(folder_path, files[complete_event_bag_ind_list[q][p]])
                    print("Path: ", old_A_file_path)
                    new_A_file_path = os.path.join(event_new_path, files[complete_event_bag_ind_list[q][p]])

                    shutil.copy2(old_A_file_path, new_A_file_path)
            csv_file_path = event_new_path +'/'+ "event_timestamp.csv"
            event_name = new_folder_name + '_' + str(q+1)

            # Create the CSV file and write the data
            with open(csv_file_path, mode='w', newline='') as csv_file:
                fieldnames = ['Event name','Event Start Time', 'Event Stop Time', 'Start Timestamp', 'Stop Timestamp']
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow({'Event name': event_name,'Event Start Time': video_start_time_str, 'Event Stop Time': video_stop_time_str, 'Start Timestamp': event_begin_time, 'Stop Timestamp': event_finish_time})

            log.debug("Event start and end time saved to {}".format(csv_file_path))

    subfolder_files = os.listdir(folder_path)
    for file in subfolder_files:
        if file.endswith(".bag"):
            file_path = os.path.join(folder_path, file)
            os.remove(file_path)

def extract_images_from_bag(input_folder, output_folder, is_camera_flipped_vert, is_camera_flipped_hor, start_time, stop_time):

    #Create the output folder if it doesn't exist. If it does exist, then delete all the files in the folder
    os.makedirs(output_folder, exist_ok=True)
    session_count = 0
    for (root, dirs, files) in os.walk(input_folder):
        for file in sorted(files):
            if file.endswith(".bag"):
                bag_number = (file.split('_')[2]).split('.')[0]
                bag = rosbag.Bag(os.path.join(root, file))
                for (topic, msg, t) in bag.read_messages():

                    time_milli = t.to_nsec() / 1e9
                    timestamp_obj = datetime.fromtimestamp(time_milli)
                    if timestamp_obj < start_time:
                        # print("Skipping frame")
                        continue
                    if timestamp_obj > stop_time:
                        # print("Stopping frame extraction for the bag")
                        break
                    bridge = CvBridge()
                    try:
                        cv_img = bridge.compressed_imgmsg_to_cv2(msg)
                        if is_camera_flipped_hor:
                            cv_img = cv2.flip(cv_img, 1)
                        if is_camera_flipped_vert:
                            cv_img = cv2.flip(cv_img, 0)
                    except CvBridgeError as e:
                        log.error(e)
                        continue


                    # if flipped:
                    #     cv_img = cv2.flip(cv_img, -1)
                    # else:
                    #     cv_img = cv2.flip(cv_img, -1)
                    session_count += 1
                    filename = os.path.join(output_folder, f'bag_{bag_number}_frame_{session_count}_{t}.png')
                    # print(filename)
                    cv2.imwrite(filename, cv_img)
                bag.close()
    return 
# Function to extract timestamp from a filename
def extract_timestamp_from_filename(filename):
    match = re.search(r"bag_\d+_frame_\d+_(\d+)", filename)    
    if match:
        return int(match.group(1))/10e5
    return None

def sync_primary_and_secondary_images(primary_image_folder_name,primary_image_folder_path, secondary_image_folder_path_name,secondary_image_folder_path, parent_folder):

    #Take the first image from the primary image folder and the secondary image folder and compare their timestamps.
    #If the timestamp of one of the image is greater than the other image by 33ms, then use the previous image of the image with the greater timestamp to fill the missing image. 
    #Give the image the timestamp of the image with the smaller timestamp. Also note the generated frame in table with the frame name and the whether it was for primary or secondary image
    #Repeat the process for the rest of the images in the primary and secondary image folders
    log.debug(primary_image_folder_name)
    log.debug(secondary_image_folder_path_name)
    primary_image_files = [f for f in os.listdir(primary_image_folder_path) if f.lower().endswith(".png")]
    secondary_image_files = [f for f in os.listdir(secondary_image_folder_path) if f.lower().endswith(".png")]
    primary_image_files = sorted(primary_image_files, key=lambda filename: extract_timestamp_from_filename(filename))
    secondary_image_files = sorted(secondary_image_files, key=lambda filename: extract_timestamp_from_filename(filename))
    primary_image_count = 1
    secondary_image_count = 1
    prim_generated_image_count = 0
    sec_generated_image_count = 0
    prev_primary_image = None
    prev_secondary_image = None
    prev_primary_image_time = 0
    prev_secondary_image_time = 0
    primary_folder_generated_frames_list = []
    secondary_folder_generated_frames_list = []
    log.debug(len(primary_image_files))
    log.debug(len(secondary_image_files))
    total_time_diff = 0
    while primary_image_count < len(primary_image_files) and secondary_image_count < len(secondary_image_files):
        primary_image_filename = primary_image_files[primary_image_count]
        secondary_image_filename = secondary_image_files[secondary_image_count]
        primary_image_path = os.path.join(primary_image_folder_path, primary_image_filename)
        secondary_image_path = os.path.join(secondary_image_folder_path, secondary_image_filename)
        primary_image_time = extract_timestamp_from_filename(primary_image_filename)
        secondary_image_time = extract_timestamp_from_filename(secondary_image_filename)
        time_diff = primary_image_time - secondary_image_time
        primary_time_diff = primary_image_time - prev_primary_image_time
        total_time_diff += abs(time_diff)
        # print("Primary time diff:", primary_time_diff)
        # print("prim:",primary_image_filename, "sec:",secondary_image_filename, "time diff:", time_diff)

        if time_diff <= 33 and time_diff >= -33:
            primary_image_count += 1
            prev_primary_image_time = primary_image_time
            secondary_image_count += 1
        elif time_diff > 33:
            #Generate a frame for the primary image
            secondary_image_count += 1
            generated_frame_time = int(secondary_image_time*1000000)
            # generated_frame_time = f'{generated_frame_time:.18f}'
            filename = os.path.join(primary_image_folder_path, f'bag_{primary_image_filename.split("_")[1]}_frame_{primary_image_filename.split("_")[3]}_{generated_frame_time}.png')
            # print(filename + ": Generated frame Primary")
            prim_generated_image_count += 1
            prev_img = cv2.imread(primary_image_path)
            cv2.imwrite(filename, prev_img)
            primary_folder_generated_frames_list.append(filename)
        elif time_diff < -33:
            #Generate a frame for the secondary image
            primary_image_count += 1
            prev_primary_image_time = primary_image_time
            generated_frame_time = int(primary_image_time*1000000)
            filename = os.path.join(secondary_image_folder_path, f'bag_{secondary_image_filename.split("_")[1]}_frame_{secondary_image_filename.split("_")[3]}_{generated_frame_time}.png')
            # print(filename + ": Generated frame Secondary")
            sec_generated_image_count += 1
            prev_img = cv2.imread(secondary_image_path)
            cv2.imwrite(filename, prev_img)
            secondary_folder_generated_frames_list.append(filename)
    # Store the primary_folder_generated_frames_list and secondary_folder_generated_frames_list in a csv file
    # Create the CSV file and write the data
    with open(os.path.join(primary_image_folder_path, primary_image_folder_name+ "_generated_frames.csv"), mode='w', newline='') as csv_file:
        fieldnames = ['Filename']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for filename in primary_folder_generated_frames_list:
            writer.writerow({'Filename': filename})
    with open(os.path.join(secondary_image_folder_path, secondary_image_folder_path_name+"_generated_frames.csv"), mode='w', newline='') as csv_file:
        fieldnames = ['Filename']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for filename in secondary_folder_generated_frames_list:
            writer.writerow({'Filename': filename})
    log.debug(primary_image_count)
    log.debug(secondary_image_count)
    log.debug('Primary generated frames: {}'.format(prim_generated_image_count))
    log.debug('Secondary generated frames: {}'.format(sec_generated_image_count))
    log.debug('Avg time difference: {}'.format(total_time_diff/primary_image_count))

def extract_gps_to_csv(input_folder,target_folder, start_time, stop_time):

    listOfBagFiles = [f for f in os.listdir(input_folder) if f[-4:] == ".bag"]  # get list of only bag files in current dir.
    numberOfFiles = str(len(listOfBagFiles))
    log.debug("Reading " + numberOfFiles + " bagfile(s) in source directory:")
    count = 0
    for bagFile in sorted(listOfBagFiles):
        count += 1
        log.debug("Reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile)
        bag = rosbag.Bag(input_folder+"/" + bagFile)
        listOfTopics = bag.get_type_and_topic_info()[1].keys()
        os.makedirs(target_folder,exist_ok=True)
        for topicName in listOfTopics:
            
            # Create a new CSV file for each topic
            filename = os.path.join(target_folder, topicName.replace('/', '_slash_') + '.csv')
            # print("Writing topic " + topicName + " to " + filename)
            firstIteration = is_empty_csv(filename) 
            with open(filename, 'a+', newline='') as csvfile:
                filewriter = csv.writer(csvfile, delimiter=',')
                 # allows header row
                for subtopic, msg, t in bag.read_messages(topicName):
                    time_milli = t.to_nsec() / 1e9
                    timestamp_obj = datetime.fromtimestamp(time_milli)
                    if timestamp_obj < start_time:
                        # print("Skipping frame")
                        continue
                    if timestamp_obj > stop_time:
                        # print("Stopping frame extraction for the bag")
                        break
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
                        if topicName == "/tcpvel":
                            headers.append("Vehicle Speed")
                        filewriter.writerow(headers)
                        firstIteration = False

                    values = [str(t)]  # first column will have rosbag timestamp
                    for pair in instantaneousListOfData:
                        if len(pair) > 1:
                            values.append(pair[1])
                    if topicName == "/tcpvel":
                        values.append(get_vehicle_speed(values[9], values[10]))
                    filewriter.writerow(values)
        bag.close()
    log.debug("Done reading all " + numberOfFiles + " bag files.")

def get_event_start_stop_time(input_folder):
    # Read event_timestamp.csv from camera_input_folder and get the start and stop time of the session from the first row in the column named Start Timestamp and Stop Timestamp
    event_timestamp_csv_path = os.path.join(input_folder, "event_timestamp.csv")
    event_timestamp_df = pd.read_csv(event_timestamp_csv_path)
    #Read start and stop time as datetime objects
    start_time = datetime.strptime(event_timestamp_df["Start Timestamp"][0], '%Y-%m-%d %H:%M:%S')
    stop_time = datetime.strptime(event_timestamp_df["Stop Timestamp"][0], '%Y-%m-%d %H:%M:%S')
    return start_time, stop_time

def is_empty_csv(path):
    if not os.path.exists(path):
        return True
    with open(path) as csvfile:
        reader = csv.reader(csvfile)
        for i, _ in enumerate(reader):
            if i:  # found the second row
                return False
    return True

def get_vehicle_speed(x_vel, y_vel):
    return math.sqrt(float(x_vel)**2 + float(y_vel)**2)

def processed_video_name(image_folder_name):
    if image_folder_name == 'images1':
        image_processed_folder_name = 'video_front'
    elif image_folder_name == 'images2':
        image_processed_folder_name = 'video_driver'
    elif image_folder_name == 'images3':
        image_processed_folder_name = 'video_right'
    elif image_folder_name == 'images4':
        image_processed_folder_name = 'video_left'
    return image_processed_folder_name

def get_video_file_name(data_classification_folder_type):
    if data_classification_folder_type == 'images1':
        image_processed_folder_name = 'video_front.mp4'
    elif data_classification_folder_type == 'images2':
        image_processed_folder_name = 'video_driver.mp4'
    elif data_classification_folder_type == 'images3':
        image_processed_folder_name = 'video_right.mp4'
    elif data_classification_folder_type == 'images4':
        image_processed_folder_name = 'video_left.mp4'
    return image_processed_folder_name

def image_processed_folder_name(image_folder_name):
    if image_folder_name == 'images1':
        image_processed_folder_name = 'image_front'
    elif image_folder_name == 'images2':
        image_processed_folder_name = 'image_driver'
    elif image_folder_name == 'images3':
        image_processed_folder_name = 'image_right'
    elif image_folder_name == 'images4':
        image_processed_folder_name = 'image_left'
    return image_processed_folder_name

# Function to extract timestamp from a filename
def extract_timestamp(filename):
    match = re.search(r"bag_\d+_frame_\d+_(\d+)", filename)    
    if match:
        return int(match.group(1))
    return None

def extract_images_to_video(input_bag_folder, output_video_path, data_classification_folder_type):
    # Initialize video writer
    codec = cv2.VideoWriter_fourcc('M','J','P','G')  # Use appropriate codec
    fps = VIDEO_FRAMERATE  # Frames per second
    video_writer = None
    os.makedirs(output_video_path,exist_ok=True)
    video_filename = os.path.join(output_video_path, processed_video_name(data_classification_folder_type) + '.mp4')
    png_files = [filename for filename in os.listdir(input_bag_folder) if filename.endswith(".png") and filename.startswith("bag")]
    sorted_png_files = sorted(png_files, key=lambda filename: extract_timestamp(filename))
    for img_filename in sorted_png_files:
        img_path = os.path.join(input_bag_folder, img_filename)
        # print("Processing:", img_path)
        frame = cv2.imread(img_path)
        # frame = cv2.flip(frame, 0)
        if video_writer is None:
            height, width, _ = frame.shape
            video_writer = cv2.VideoWriter(video_filename, codec, fps, (width, height))
        video_writer.write(frame)

    if video_writer is not None:
        video_writer.release()
        log.debug("Video conversion complete. Output file: {}".format(video_filename))
    else:
        log.error("No bag files found for conversion.")

def get_lane_deviation(left_lane_dist, right_lane_distance):
    half_lane_width = 180
    if left_lane_dist == 500 and right_lane_distance == 500:
        deviation = 500
    elif left_lane_dist == 500:
       deviation = right_lane_distance - half_lane_width
    elif right_lane_distance == 500:
        deviation = half_lane_width - left_lane_dist
    else:
        deviation = (right_lane_distance - left_lane_dist)/2
    return deviation