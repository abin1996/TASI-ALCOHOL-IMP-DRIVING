
import os, os.path, shutil
from datetime import datetime,timedelta
import csv
import rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
import pytz
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


    print(new_folder_name,':')
    print('Event bag index:', event_bag_ind_list)

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

    print('Complete bag index:', complete_event_bag_ind_list)
    # create the csv file for each session
    event_timestamp_paired = [] # Initialize the list for paried timestamps. The list would have the format:[[event1_start_timestamp,event1_end_timestamp],[e2_start,e2_end],...]

    for k in range(0, len(filtered_timestamps), 2):
        sub_lst = filtered_timestamps[k:k + 2]  # Get two elements from the current position
        event_timestamp_paired.append(sub_lst)  # Append the sublist to the result list

    for q in range(0,len(complete_event_bag_ind_list)):
        bag_start_time = bag_time_list[complete_event_bag_ind_list[q][0]][0]

        if len(event_timestamp_paired[q]) == 2:
            event_begin_time, event_finish_time = event_timestamp_paired[q]
            print('q:',q)

            # Calculate the time differences
            video_start_time = event_begin_time - bag_start_time
            video_stop_time = event_finish_time - bag_start_time

            # Format the time differences as strings
            video_start_time_str = str(video_start_time)
            video_stop_time_str = str(video_stop_time)
            print('Bag start time:',bag_start_time)
            print('Event start time:', event_begin_time)
            print('Event end time:', event_finish_time)
            print('..............')
            print('Video start time:',video_start_time_str)
            print('Video end time:', video_stop_time_str)

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
                fieldnames = ['Event name','Event Start Time', 'Event Stop Time']
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow({'Event name': event_name,'Event Start Time': video_start_time_str, 'Event Stop Time': video_stop_time_str})
            
            print(f"Event start and end time saved to '{csv_file_path}'.")
            print('..............')

    subfolder_files = os.listdir(folder_path)
    for file in subfolder_files:
        if file.endswith(".bag"):
            file_path = os.path.join(folder_path, file)
            os.remove(file_path)
    print('------------------------------------------------------------')

def extract_images_from_bag(bag, output_folder, flipped, bag_number, session_count, prev_frame, prev_frame_time):
    total_missing_frames = 0
    frame_gaps_list = []
    #Time between each fram is 33 ms (30 fps). If the time between last and current frame is more than 35 ms, then there is a missing frame. 
    #When there is a missing frame, use the last frame to fill the missing frame. Record the number of missing frames in a list as a tuple (start_of_missing_frame, end_of_missing_frame, number_of_missing_frames)
    missing_frame_count = 0
    
    #Create the output folder if it doesn't exist. If it does exist, then delete all the files in the folder
    os.makedirs(output_folder, exist_ok=True)

    for (topic, msg, t) in bag.read_messages():
        bridge = CvBridge()
        try:
            cv_img = bridge.compressed_imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            print(e)
            continue
        if flipped:
            cv_img = cv2.flip(cv2.flip(cv_img, 1), 0)
        
        if prev_frame_time != 0:
            time_diff = float(t.to_sec() - prev_frame_time.to_sec())
            # print(time_diff)
            if time_diff > 0.044:
                missing_frame_count = int(time_diff/0.044)
                total_missing_frames += missing_frame_count
                frame_gaps_list.append((bag_number, session_count, missing_frame_count, time_diff))
                for i in range(missing_frame_count):
                    session_count += 1
                    generated_frame_time = datetime.utcfromtimestamp(prev_frame_time.to_sec()) + timedelta(milliseconds=33*(i+1))
                    generated_frame_time = rospy.Time.from_sec(generated_frame_time.replace(tzinfo=pytz.UTC).timestamp())
                    filename = os.path.join(output_folder, f'bag_{bag_number}_frame_{session_count}_{generated_frame_time}.png')
                    print(filename + ": Generated frame")
                    cv2.imwrite(filename, prev_frame)
                    
        session_count += 1
        filename = os.path.join(output_folder, f'bag_{bag_number}_frame_{session_count}_{t}.png')
        print(filename)
        cv2.imwrite(filename, cv_img)
        prev_frame_time = t
        prev_frame = cv_img
    
    return frame_gaps_list, session_count,  prev_frame, prev_frame_time, total_missing_frames

def image_processed_folder_name(image_folder_name):
    if image_folder_name == 'images1':
        image_processed_folder_name = 'image_driver'
    elif image_folder_name == 'images2':
        image_processed_folder_name = 'image_front'
    elif image_folder_name == 'images3':
        image_processed_folder_name = 'image_right'
    elif image_folder_name == 'images4':
        image_processed_folder_name = 'image_left'
    return image_processed_folder_name