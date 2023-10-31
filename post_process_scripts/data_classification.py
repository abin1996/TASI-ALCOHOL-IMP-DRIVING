from datetime import datetime,timedelta
import os, os.path, shutil
import time
import csv

def timestamp_to_date(timestamp):
    # convert the timestamp to a datetime object in the local timezone
    dt_object = datetime.fromtimestamp(timestamp)
    time = dt_object.time()
    # using __str__()
    # time_string = time.__str__()

    # hr = time_string.split(':')[0]
    # min = time_string.split(':')[1]
    # sec = time_string.split(':')[2]

    # return (hr,min,sec)
    return dt_object

def event_timestamp(filename):
    # opens the file, reads and stores each line into a list
    with open(filename) as file:
        lines = file.readlines()

    index_button_list=[]
    for i in range(7,len(lines),9): # get the line number of button in each sequence
        index_button_list.append(i)
    # print('button_list',index_button_list)

    index_nonzero_button_list = [] # the list contains all button log that is of interest
    for m in index_button_list:
        if lines[m] == "buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n" \
                or lines[m] == "buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n" \
                or lines[m] == "buttons: [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]\n" \
                or lines[m] == "buttons: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]\n":
            index_nonzero_button_list.append(m)

    # print('nonzero_button_list',index_nonzero_button_list)

    # index_zero_button_list = [] # the list contains all button log that is not of interest
    # for m in index_button_list:
    #     if lines[m] != "buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n" \
    #             or lines[m] != "buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"\
    #             or lines[m] != "buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"\
    #             or lines[m] != "buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n":
    #         index_zero_button_list.append(m)
    # print('zero_button_list',index_zero_button_list)

    target_line_index_list = []
    for p in index_nonzero_button_list:
        del_start = p-7 # the starting line number of zero button sequence
        del_end = p+1 # the ending line number of zero button sequence
        for i in range(del_start,del_end+1,1):
            target_line_index_list.append(i)

    new_filename ='event_timestamp.txt'
    # Write file
    with open(new_filename, "w") as file:
        # iterate each line
        for number, line in enumerate(lines):
            # delete lines from the saved list
            if number in target_line_index_list:
                file.write(line)

    # read file
    with open(new_filename, 'r') as fp:
        # read an store all lines into list
        new_lines = fp.readlines()

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

    return event_A_timestamp_list,event_B_timestamp_list,event_X_timestamp_list,event_Y_timestamp_list


def copy_files_to_subfolders(source_folder, subfolder_names):
    # Ensure the source folder exists
    if not os.path.exists(source_folder):
        print(f"The source folder '{source_folder}' does not exist.")
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

    # print("Files copied to subfolders successfully.")


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

    return filtered_timestamps

def file_recategory(timestamp_list, org_folder_name, new_folder_name):
    event_timestamp_list = timestamp_list
    folder_path = org_folder_name + '/'+new_folder_name

    # timestamp_dt_list=[]

    # for timestamp in event_timestamp_list:
    #     hour, minute, second = timestamp
    #     # Define the timestamp as a string
    #     timestamp_str = hour+':' + minute +':'+ second
    #     # Convert the timestamp to a datetime object
    #     timestamp_dt = datetime.strptime(timestamp_str, "%H:%M:%S")
    #     timestamp_dt_list.append(timestamp_dt)

    # # Initialize the list for the filtered timestamps
    # filtered_timestamps = []

    # # Iterate through the timestamps and keep the first timestamp with a gap of more than 2 seconds
    # for i in range(len(timestamp_dt_list) - 1):
    #     current_time = timestamp_dt_list[i]
    #     next_time = timestamp_dt_list[i + 1]
    #     time_difference = (next_time - current_time).total_seconds()

    #     if time_difference <= 2:
    #         filtered_timestamps.append(current_time)
    # Filter out the single misinput timestamp and duplicated timestamps in timestamp_dt_list

    filtered_timestamps = filtered_event_timestamp(event_timestamp_list)
    # print("Filtered timestamps:", filtered_timestamps)new_directory = "/home/iac_user/DATA_COLLECTION(DO NOT DELETE)/" + SUBJECT_ID + '/' + ALCOHOL_LEVEL + "/26-10-23_12-27-01" + "/event_signal"



    files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    files.sort()
    bag_time_list=[]

    for file in files:
        # bag_year = file.split('-')[0]
        # bag_month = file.split('-')[1]
        # bag_date = file.split('-')[2]
        # bag_hour = file.split('-')[3]
        # bag_min = file.split('-')[4]
        bag_sec_string = file.split('-')[5]
        # bag_sec = bag_sec_string.split('_')[0]

        bag_ind_string = bag_sec_string.split('_')[1]

        bag_ind = bag_ind_string.split('.')[0]

        datetime_str = file.split("_")[1]

        datetime_obj = datetime.strptime(datetime_str,"%Y-%m-%d-%H-%M-%S")

        bag_time_list.append((datetime_obj,int(bag_ind)))
        # bag_time_list.append((bag_hour,bag_min,bag_sec,bag_ind))
    bag_time_list.sort(key=lambda a: a[1])
    # print("Bag time List ", bag_time_list)
    event_bag_ind_list=[]

    for index, timestamp in enumerate(filtered_timestamps):
        # hour, minute, second = timestamp
        # Define the timestamp as a string
        # timestamp_str = hour+':' + minute +':'+ second
        # Convert the timestamp to a datetime object
        # timestamp = datetime.strptime(timestamp_str, "%H:%M:%S")
        # timestamp = time_info[0]

        for i in range(0,len(bag_time_list)):
            # print("Bag Time List: ",bag_time_list[i])
            if i < len(bag_time_list)-1:

                start_time = bag_time_list[i][0]
                # print("Start Time: ", start_time)
                end_time = bag_time_list[i+1][0]
                # print("End Time: ", end_time)


                # Check if the timestamp is within the time slot
                if start_time <= timestamp < end_time:
                    # print(f"The timestamp {timestamp_str} is within the time slot.")
                    event_bag_ind_list.append(int(bag_time_list[i][1]))

                    # event_bag_ind_list.append(int(bag_time_list[i][3])+1)
                    break
            else:

                start_time = bag_time_list[i][0]
                end_time = start_time + timedelta(minutes=1)

                # Check if the timestamp is within the time slot
                if start_time <= timestamp < end_time:
                    # print(f"The timestamp {timestamp_str} is within the time slot.")
                    event_bag_ind_list.append(int(bag_time_list[i][1]))

                    # event_bag_ind_list.append(int(bag_time_list[i][3])+1)
                    break


    print(new_folder_name,':')
    print('event bag index:', event_bag_ind_list)

    # Initialize an empty list to store the sublists
    event_bag_ind_list_paired = []

    # Iterate through the list in steps of 2 [start_bag_ind, end_bag_ind]
    for i in range(0, len(event_bag_ind_list), 2):
        sublist = event_bag_ind_list[i:i + 2]  # Get two elements from the current position
        event_bag_ind_list_paired.append(sublist)  # Append the sublist to the result list


    # Print the resulting list of sublists
    # print('paired bag ind',event_bag_ind_list_paired)

    complete_event_bag_ind_list = []
    for j in range(len(event_bag_ind_list_paired)):
        lst = event_bag_ind_list_paired[j]
        complete_lst = insert_missing_numbers(lst)
        complete_event_bag_ind_list.append(complete_lst)

    print('complete bag index:', complete_event_bag_ind_list)

    global event_new_path

       # create the csv file for each session
    event_timestamp_paired = [] # Initialize the list for paried timestamps. The list would have the format:[[event1_start_timestamp,event1_end_timestamp],[e2_start,e2_end],...]

    for j in range(0, len(filtered_timestamps), 2):
        sub_lst = filtered_timestamps[i:i + 2]  # Get two elements from the current position
        event_timestamp_paired.append(sub_lst)  # Append the sublist to the result list


    for q in range(0,len(complete_event_bag_ind_list)):
        bag_start_time = bag_time_list[complete_event_bag_ind_list[k][0]][0]

        event_begin_time, event_finish_time = event_timestamp_paired[k]

        # Calculate the time differences
        video_start_time = event_begin_time - bag_start_time
        video_stop_time = event_finish_time - bag_start_time

        # Format the time differences as strings
        video_start_time_str = str(video_start_time)
        video_stop_time_str = str(video_stop_time)

        print('Video start time:',video_start_time_str)
        print('Video end time:', video_stop_time_str)




        subtest_ind = q + 1
        # print("q: ",q)

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

    subfolder_files = os.listdir(folder_path)

    for file in subfolder_files:
        if file.endswith(".bag"):
            file_path = os.path.join(folder_path, file)
            os.remove(file_path)

 

    for k in range(len(complete_event_bag_ind_list)):
        
        
        # Define the CSV file path

        #TODO: CHANGE THE FOLDER TO THE VIDEO OUTPUT FOLDER
        csv_file_path = event_new_path +'/'+ "event_timestamp.csv"

        # Create the CSV file and write the data
        with open(csv_file_path, mode='w', newline='') as csv_file:
            fieldnames = ['Event Start Time', 'Event Stop Time']
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            
            writer.writeheader()
            writer.writerow({'Event Start Time': video_start_time_str, 'Event Stop Time': video_stop_time_str})

        print(f"Event time differences saved to '{csv_file_path}'.")


    print('------------------------------------------------------------')


start_time = time.time()
# Define the path to the new working directory
# TODO: change the directory Ffor the event signal file
# new_directory = "/media/iac_user/ImDrive_Org/Org-Data/TestLL/Baseline/only_driving/17-10-23_10-50-24/event_signal"

#10/24 baseline
new_directory = "/home/iac_user/DATA_COLLECTION/SubjectAnn/Baseline/24-10-23_12-43-25" + "/event_signal"

#10/24 70-alcohol
# new_directory = "/home/iac_user/DATA_COLLECTION(DO NOT DELETE)/Subject01/70-Alcohol/26-10-23_11-41-54" + "/event_signal"


#TODO:NEED TO CHANGE THE SUBJECT ID FOR EACH DATA COLLECTION!  

SUBJECT_ID = 'Subject01'

ALCOHOL_LEVEL = 'Baseline'

# ALCOHOL_LEVEL = '70-Alcohol'

# ALCOHOL_LEVEL = '80-Alcohol'


#10/24 80-alcohol
# new_directory = "/home/iac_user/DATA_COLLECTION(DO NOT DELETE)/" + SUBJECT_ID + '/' + ALCOHOL_LEVEL + "/26-10-23_12-27-01" + "/event_signal"

# Change the working directory
os.chdir(new_directory)

joystick_filename = 'joystick.txt'

#TODO: source_folder needs to enumerate all four image folders and the gps folder

source_folder = "/home/iac_user/DATA_COLLECTION/SubjectAnn/Baseline/24-10-23_12-43-25/gps"

#Baseline
# source_folder = '/home/iac_user/DATA_COLLECTION(DO NOT DELETE)/' + SUBJECT_ID + '/' + ALCOHOL_LEVEL + '/26-10-23_10-21-30/images4'

# 70-alcohol
# source_folder = '/media/iac_user/ImDrive_Bck/SubjectAnn/70-Alcohol/24-10-23_13-49-28/images4'
# source_folder = '/home/iac_user/DATA_COLLECTION(DO NOT DELETE)/' + SUBJECT_ID + '/' + ALCOHOL_LEVEL + '/26-10-23_11-41-54/images4'

# 80-alcohol
# source_folder = '/home/iac_user/DATA_COLLECTION(DO NOT DELETE)/' + SUBJECT_ID + '/' + ALCOHOL_LEVEL + '/26-10-23_12-27-01/images4'

# Based on the joystick instruction:
# Event A is Eye tracking
# Event B is Driving backward
# Event X is Parking
# Event Y is Driving forward

subfolder_names = ["Eye tracking", "Driving backward", "Parking", "Driving forward"]

copy_files_to_subfolders(source_folder, subfolder_names)
timestamp_lists = event_timestamp(joystick_filename)
# print('event timestamp list',timestamp_lists)

org_folder_name = source_folder

for i in range(0,len(subfolder_names)):
    # if subfolder_names[i] != "Driving backward":
    #     continue
    timestamp_list = timestamp_lists[i]
    new_folder_name = subfolder_names[i]
    file_recategory(timestamp_list, org_folder_name, new_folder_name)

print('Data recategory finished')

end_time = time.time()

execution_time = end_time - start_time

print('Execution time',execution_time)