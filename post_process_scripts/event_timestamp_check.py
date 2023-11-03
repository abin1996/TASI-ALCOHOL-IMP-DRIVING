import os
from datetime import datetime,timedelta


def timestamp_to_date(timestamp):
    # convert the timestamp to a datetime object in the local timezone
    dt_obj = datetime.fromtimestamp(timestamp)

    return dt_obj

def paired_filtered_event_timestamp(timestamp_list):
    # Initialize the list for the filtered timestamps
    filtered_timestamps = []

    # Iterate through the timestamps and keep the first timestamp with a gap of more than 2 seconds
    for i in range(len(timestamp_list) - 1):
        current_time = timestamp_list[i]
        next_time = timestamp_list[i + 1]
        time_difference = (next_time - current_time).total_seconds()

        if time_difference <= 2:
            filtered_timestamps.append(current_time)

    # Remove the same mistaken input when the time difference between the two timestamps are less than or equal to 1s
    # Which also means if pushing the wrong button, need to wait for 2s to double click again
    # for j in range(len(filtered_timestamps)-1):
    #     current_timestamp = filtered_timestamps[j]
    #     next_timestamp = filtered_timestamps[j+1]
    #     timestamp_difference = (next_timestamp - current_timestamp).total_seconds()

    #     if timestamp_difference <= 1:
    #         filtered_timestamps.remove(next_timestamp)
    filtered_timestamps = [timestamp for j, timestamp in enumerate(filtered_timestamps) if j == 0 or (timestamp - filtered_timestamps[j-1]).total_seconds() > 1]

    paired_filtered_list = [filtered_timestamps[i:i + 2] for i in range(0, len(filtered_timestamps), 2)]

    return paired_filtered_list

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

def timestamp_check(EVENT_NAME, timestamp_list,output_folder_path):


    event_timestamp_list = paired_filtered_event_timestamp(timestamp_list)


    # delete_session_name = 'Parking'
    # delete_timestamp_ind = [2,3,4]

    # if EVENT_NAME == delete_session_name:
    #     event_timestamp_list = [timestamp for i, timestamp in enumerate(event_timestamp_list) if i not in delete_timestamp_ind]


    for i in range(len(event_timestamp_list)):
        sub_session_name_str = EVENT_NAME + '_' + str(i + 1)

        if len(event_timestamp_list[i]) == 2:

            start_time,end_time = event_timestamp_list[i]
            sub_session_duration = end_time - start_time

            print('Start time of',sub_session_name_str,'is:')
            print(start_time)

            print('End time of',sub_session_name_str,'is:')
            print(end_time)

            print('Duration of',sub_session_name_str, 'is:')
            print(sub_session_duration)

        else:
            print('Duration of', sub_session_name_str, 'is:')
            print('Missing one timestamp from controller input!')
        print('-------------------------')



    print('-------------------------')


if __name__ == '__main__':

    # new_directory = "F:/SubjectAnn/Baseline/24-10-23_12-43-25/event_signal"
    # new_directory = "/home/iac_user/DATA_COLLECTION/SubjectAnn/70-Alcohol/24-10-23_13-49-28/event_signal"

    
    new_directory = "/home/iac_user/DATA_COLLECTION(DO NOT DELETE)/Subject02/Baseline/31-10-23_10-16-52" + "/event_signal"

    # Change the working directory
    os.chdir(new_directory)

    joystick_filename = 'joystick.txt'
    timestamp_lists = event_timestamp(joystick_filename)

    EVENTS = ['Eye tracking','Driving backward','Parking','Driving forward']
    SOURCE_FOLDER = None

    for i in range(len(timestamp_lists)):
        event_timestamps = timestamp_lists[i]
        timestamp_check(EVENTS[i],event_timestamps,SOURCE_FOLDER)

    print('Timestamp check finished')
