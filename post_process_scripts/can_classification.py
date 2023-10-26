import pandas as pd
import os
from datetime import datetime

def timestamp_to_date(timestamp):

    # convert the timestamp to a datetime object in the local timezone
    dt_obj = datetime.fromtimestamp(timestamp)

    # using __str__()
    time_string = dt_obj.strftime('%d-%m-%y %H:%M:%S')

    return dt_obj
    # return time_string

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


def can_recategory(source_file,timestamp_ranges,save_file_path):
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

            output_file_name = "CAN_data.csv"

            # Define the complete file path
            output_file_path = os.path.join(user_defined_directory, output_file_name)

            if not os.path.exists(user_defined_directory):
                os.makedirs(user_defined_directory)  # Create the directory if it doesn't exist

            # # Create a new CSV file for the filtered data
            filtered_data.to_csv(output_file_path, index=False)

        else:
            print('Session',str(EVENT_IND),'Missing one controller button input')

    print("CSV files have been created.")



if __name__ == '__main__':

    # new_directory is the location of the event_signal
    # new_directory = "D:/HZTs_file/Impaired-Driving/TestLL/Baseline/only_driving/17-10-23_10-50-24/event_signal"
    new_directory = "/home/iac_user/DATA_COLLECTION/SubjectAnn/70-Alcohol/24-10-23_13-49-28/event_signal"

    # Change the working directory
    os.chdir(new_directory)
    joystick_filename = 'joystick.txt'
    timestamp_lists = event_timestamp(joystick_filename)

    EVENTS = ['Eye tracking','Driving backward','Parking','Driving forward']

    # CSV_SOURCE_FILE = 'D:/HZTs_file/Impaired-Driving/Post-SubjectLL/Baseline/only_driving/17-10-23_10-50-24/can/can_test.csv'
    # output_folder_name = 'D:/HZTs_file/Impaired-Driving/Post-SubjectLL/Baseline/only_driving/17-10-23_10-50-24/can/'


    CSV_SOURCE_FILE = '/home/iac_user/DATA_COLLECTION/SubjectAnn/70-Alcohol/24-10-23_13-49-28/can/can_messages_24-10-23_13-49-28.csv'
    output_folder_name = '/home/iac_user/POST_PROCESS/SubjectAnn/70-Alcohol/'

    for i in range(len(timestamp_lists)):
        event_timestamps = timestamp_lists[i]

        filtered_list = filtered_event_timestamp(event_timestamps)

        paired_filtered_list = [filtered_list[i:i+2] for i in range(0, len(filtered_list), 2)]

        if not event_timestamps:
            print('No',EVENTS[i],'session found')
        else:
            print(EVENTS[i])
            # print(timestamp_lists[i])
            # print('Filtered list is:',filtered_list)
            print('Paired list is',paired_filtered_list)
            print('Number of sessions',len(paired_filtered_list))


            save_file_path = output_folder_name + EVENTS[i] + '/' + EVENTS[i]

            can_recategory(CSV_SOURCE_FILE, paired_filtered_list, save_file_path)
            print('---------------------------------')

