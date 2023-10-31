import subprocess
import time
import os
import signal
import PySimpleGUI as sg
import threading
import re
import datetime
import argparse
from pathlib import Path

from record_brac_can_messages_window_out import record_brac
from check_data_collection_system import check_system_connection
DATA_COLLECTION_FOLDER_NAME = ''

HOME_DIR="/home/iac_user/"
DATA_COLLECTION_DIR="DATA_COLLECTION/"
BRAC_DIR="brac/"
BRAC_DBC='/home/iac_user/data_collection_scripts/dadss_breath_sensor_fixed.dbc'

CHECK_SYSTEM_CONNECTION = "CHECK DEVICE CONNECTION"

START_RECORD_AUDIO_BUTTON = "AUDIO TEST: START RECORDING"
STOP_AUDIO_RECORD_BUTTON = "AUDIO TEST: STOP RECORDING"

BEFORE_DRIVE_START_BRAC_BUTTON = "BEFORE DRIVE BREATH SENSOR: START RECORDING"
AFTER_DRIVE_START_BRAC_BUTTON = "AFTER DRIVE BREATH SENSOR: START RECORDING"
START_RECORDING_DRIVING_BUTTON = "EYE TRACKING AND DRIVING TEST: START RECORDING"
STOP_RECORDING_DRIVING_BUTTON = "EYE TRACKING AND DRIVING TEST: STOP RECORDING"


ROSCORE_START_SCRIPT = "roscore"
START_DRIVERS_SCRIPT = "/home/iac_user/data_collection_scripts/start_drivers.sh"
START_RECORDING_FOR_DRIVING_SCRIPT = "/home/iac_user/data_collection_scripts/start_recording_cam_gps_joy.sh"
CHECK_RECORDING_FOR_DRVING_SCRIPT = "/home/iac_user/data_collection_scripts/check_recording_v2.sh"
START_AUDIO_RECORDING_SCRIPT = "/home/iac_user/data_collection_scripts/start_audio_recording.sh"
START_BRAC_RECORDING_SCRIPT = "/home/iac_user/data_collection_scripts/start_can_recording_brac.sh"

WINDOW_OUTPUT_TEXT= "*"
# Define the layout of the GUI
#The buttons for BRAC and AUDIO should be on the left and the buttons for EYE TRACKING AND DRIVING should be on the right
layout = [
    [
        sg.Column([
            [sg.Frame("Data Collection Information", [[sg.Text("Subject: \nSession Name:",font=('Helvetica', 20), key='-TEXT1-', text_color='black')]])],
            [sg.Button(CHECK_SYSTEM_CONNECTION,button_color="brown",font=('Helvetica', 20))],
            [sg.Button(BEFORE_DRIVE_START_BRAC_BUTTON,button_color="darkblue",font=('Helvetica', 20))],
            # [sg.VPush()],
            [sg.Button(START_RECORD_AUDIO_BUTTON,button_color="green",font=('Helvetica', 20))],
            [sg.Button(STOP_AUDIO_RECORD_BUTTON,button_color="red", font=('Helvetica', 20))],
            # [sg.VPush()],
            [sg.Button(START_RECORDING_DRIVING_BUTTON,button_color="green",font=('Helvetica', 20))],
            [sg.Button(STOP_RECORDING_DRIVING_BUTTON,button_color="red",font=('Helvetica', 20))],
            [sg.Button(AFTER_DRIVE_START_BRAC_BUTTON,button_color="darkblue",font=('Helvetica', 20))],
            [sg.Button("Exit", font=('Helvetica', 20))],]),
        sg.Column([[sg.Frame("Devices Connection Status", [[sg.Multiline(disabled=True,size=(50, 10), font=("Courier New", 25), key="-DEVICE-", background_color="black", text_color="white")]],expand_x=True,expand_y=True)]], expand_x=True,expand_y=True)
    ],
    [sg.Frame("Output", [[sg.Multiline(disabled=True,size=(150, 30), font=("Courier New", 25), key="-OUTPUT-", background_color="black", text_color="white")]],expand_x=True,expand_y=True)],
 
    
]

# Create the window
window = sg.Window("TASI DATA COLLECTION", layout,resizable=True)
# Global variable to store the running processes in reverse order
RUNNING_PROCESSES = []


def get_rec_file_name(string):
    pattern = r"\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}"
    match = re.search(pattern, string)
    # Check if a match was found and print the timestamp
    if match:
        return match.group()
    else:
        return ''


def get_current_time():
    current_time = datetime.datetime.now()
    return current_time.strftime('%d-%m-%y_%H-%M-%S')

# Function to execute the Bash command in a new terminal and capture output
def execute_bash_command_in_terminal(command, command_type,is_shell=False):
    print("Command:",command)
    process = subprocess.Popen(
        command,
        shell=is_shell,
        stdout=subprocess.PIPE,
        preexec_fn=os.setsid,  # Create a new process group
        universal_newlines=True,
    )
    RUNNING_PROCESSES.insert(0, (command_type,process))  # Add process at the beginning
    print(RUNNING_PROCESSES)

# Function to kill all running processes and threads in reverse order
def kill_running_processes(RUNNING_PROCESSES):
    for process in RUNNING_PROCESSES:
        print("Killing: ",process[0])
        print(process[1].pid)
        os.killpg(os.getpgid(process[1].pid), signal.SIGTERM)  # Send the signal to the entire process group
        time.sleep(1)
    RUNNING_PROCESSES = []

def kill_running_processes_name(process_name,RUNNING_PROCESSES):
    remove_process=None
    for process in RUNNING_PROCESSES:
        if process[0] == process_name:
            remove_process = process
            print("Killing: ",process[0])
            os.killpg(os.getpgid(process[1].pid), signal.SIGTERM)  # Send the signal to the entire process group
            time.sleep(0.5)
            break
    RUNNING_PROCESSES.remove(remove_process)

def call_bash_script(script_path, window_text, subject_id, sessionName):
    try:
        # Run the Bash script and capture the output
        window['-OUTPUT-'].update(window_text)
        # print("-------- Recording Status --------")
        command = ["bash", script_path, subject_id]
        print(command)
        result = subprocess.run(["bash", script_path, subject_id, sessionName], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
         # Print the output
        check_rec_resp(result.stdout, window_text)
            
    except FileNotFoundError:
        window['-OUTPUT-'].update(f"Error: The script '{script_path}' was not found.")
    except subprocess.CalledProcessError as e:
        window['-OUTPUT-'].update(f"Error: The script returned a non-zero exit status. Exit code: {e.returncode}")
    except Exception as error:
        # handle the exception
        print("An exception occurred:", type(error).__name__, "-", error)

def extract_bag_number(path):
    # Extract the last part of the path and remove the file extension
    filename = path.split("/")[-1].split(".")[0]
    bag_number = filename.split("_")[-1]
    return int(bag_number)

def check_rec_resp(script_resp,window_text):
    print(script_resp)
    if not "size" in script_resp:
        window_text += "\n\nRecording status will change after a minute"
        window['-OUTPUT-'].update(window_text)
        return
    category_data = {}
    current_category = None
    # rec_file_name = get_rec_file_name(script_resp)
    # display_text = "\nRECORDING FOLDER NAME: {}".format(rec_file_name)
    # window_text += display_text
    # window['-OUTPUT-'].update("RECORDING FOLDER NAME: ",rec_file_name)
    display_text = ""
    lines = script_resp.strip().split("\n")
    for line in lines:
        if "************************" in line:
            current_category = line.strip("*")
            category_data[current_category] = {}
        else:
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            category_data[current_category][key.strip()] = value.strip()

    # print(category_data)
    for category, data in category_data.items():
        if category == "CAN-MESSAGES":
            if "size" not in data:
                data["status"] = "ERROR: CAN FILE SIZE NOT FOUND. CHECK IF CAN IS RECEIVING SIGNAL ON THE TERMINAL"
                continue
            size = float(data["size"])
            if size/1024 > 4:
                data["status"] = "ALL GOOD. CAN FILE SIZE:" +str((size / 1024 / 1024)) + "MB"
            else:
                data["status"] = "ERROR: CAN FILE SIZE TOO LOW. CHECK IF CAN IS RECEIVING SIGNAL ON THE TERMINAL"
        else:
            size = float(data["size"].split()[0])
            bag_number = extract_bag_number(data["path"])
            data['bag_number'] = bag_number
            if category == "GPS" and size > 10:
                data["status"] = "ALL GOOD. GPS FILE SIZE:" +str(size)
            elif category == "GPS" and size < 10:
                data["status"] = "ERROR: GPS FILE SIZE TOO LOW. CHECK IF GPS IS RECEIVING SIGNAL ON THE TERMINAL"
            elif category.startswith("CAMERA") and size > 20:
                data["status"] = "ALL GOOD. CAMERA FILE SIZE:" +str(size)
            elif category.startswith("CAMERA") and size < 20:
                data["status"] = "ERROR: CAMERA FILE SIZE TOO LOW. CHECsizeK THE CAMERA CONNECTIONS AND TERMINAL FOR ERRORS"
    for category, data in category_data.items():
        status = data.get('status'," ")
        bag_number = data.get('bag_number'," ")
        if status.startswith("ERROR"):
            display_text += "\n"+"!!!!!!!!!!  ERROR  !!!!!!!!!!" 
        if bag_number != " ":
            display_text += "\n" + f"{category} -- Bag Number:{bag_number}, Status: {status}"
        
        else:
            display_text += "\n" + f"{category} -- Status: {status}"
    window_text += display_text
    window['-OUTPUT-'].update(window_text)

def update_output_element(window_text, script_path, subject_id, sessionName, stop_check_status_thread):
    while True:
        if stop_check_status_thread():
            print("Closing Thread for Checking DRIVING Video Recording Status")
            break
        call_bash_script(script_path,window_text, subject_id, sessionName)
        time.sleep(30)
        # Adjust the update interval as needed

def main(subject_id, sessionName):
    # Event loop to process events and get button clicks
    global DATA_COLLECTION_FOLDER_NAME
    global RUNNING_PROCESSES
    global WINDOW_OUTPUT_TEXT
    check_counter= 0
    video_recorded_started = False
    stop_check_status_thread = False
    execute_bash_command_in_terminal([ROSCORE_START_SCRIPT],"roscore")
    time.sleep(0.5)
    #Update the text on the GUI with the subject ID and session name
    subject_data = f"Subject: {subject_id} \nSession Name: {sessionName}"
    while True:
        event, values = window.read()
        window['-TEXT1-'].update(subject_data)
        # check_counter += 1
        # ret_code = check_system_connection(window['-DEVICE-'], check_counter)
        
        if event == CHECK_SYSTEM_CONNECTION:
            check_counter += 1
            check_system_connection(window['-DEVICE-'], check_counter)
        if event in (sg.WINDOW_CLOSED, "Exit"):
            break
            
        #BREATH SENSOR EVENTS
        if event == BEFORE_DRIVE_START_BRAC_BUTTON:
            if DATA_COLLECTION_FOLDER_NAME == '':
                DATA_COLLECTION_FOLDER_NAME = get_current_time()
            check_counter += 1
            check_system_connection(window['-DEVICE-'], check_counter)

            start_time = datetime.datetime.now()
            root_brac_folder = HOME_DIR + DATA_COLLECTION_DIR + subject_id + '/' + 'combined_brac' + '/'
            root_brac_filename = root_brac_folder + 'brac_reading_all_sessions.csv'
            output_dir = Path(root_brac_folder)
            output_dir.mkdir(parents=True, exist_ok=True)

            parent_folder_brac_path = HOME_DIR + DATA_COLLECTION_DIR + subject_id +  '/' + sessionName + '/' + DATA_COLLECTION_FOLDER_NAME + '/' + BRAC_DIR 
            output_dir = Path(parent_folder_brac_path)
            output_dir.mkdir(parents=True, exist_ok=True)
            filename = parent_folder_brac_path + 'brac_{}.csv'.format(start_time.strftime('%d-%m-%y_%H-%M-%S'))
            # print("filename:",filename)
            output_thread = threading.Thread(target=record_brac, args=(filename, start_time, BRAC_DBC, window['-OUTPUT-'], subject_id, sessionName, "before", root_brac_filename), daemon=True)
            output_thread.start()  

        #BREATH SENSOR EVENTS
        if event == AFTER_DRIVE_START_BRAC_BUTTON:
            if DATA_COLLECTION_FOLDER_NAME == '':
                DATA_COLLECTION_FOLDER_NAME = get_current_time()
            start_time = datetime.datetime.now()
            root_brac_folder = HOME_DIR + DATA_COLLECTION_DIR + subject_id + '/' + 'combined_brac' + '/'
            root_brac_filename = root_brac_folder + 'brac_reading_all_sessions.csv'
            output_dir = Path(root_brac_folder)
            output_dir.mkdir(parents=True, exist_ok=True)

            parent_folder_brac_path = HOME_DIR + DATA_COLLECTION_DIR + subject_id +  '/' + sessionName + '/' + DATA_COLLECTION_FOLDER_NAME + '/' + BRAC_DIR 
            output_dir = Path(parent_folder_brac_path)
            output_dir.mkdir(parents=True, exist_ok=True)
            filename = parent_folder_brac_path + 'brac_{}.csv'.format(start_time.strftime('%d-%m-%y_%H-%M-%S'))
            # print("filename:",filename)
            output_thread = threading.Thread(target=record_brac, args=(filename, start_time, BRAC_DBC, window['-OUTPUT-'], subject_id, sessionName, "after", root_brac_filename), daemon=True)
            output_thread.start()  

        #AUDIO RECORDING EVENTS
        if event == START_RECORD_AUDIO_BUTTON:
            if DATA_COLLECTION_FOLDER_NAME == '':
                DATA_COLLECTION_FOLDER_NAME = get_current_time()
            check_counter += 1
            check_system_connection(window['-DEVICE-'], check_counter)
            execute_bash_command_in_terminal(['bash', START_AUDIO_RECORDING_SCRIPT, DATA_COLLECTION_FOLDER_NAME, subject_id, sessionName],"audio_recording",is_shell=False)
            output_text = "\nSTARTED AUDIO RECORDING IN FOLDER: {}/{}/{}".format(subject_id,sessionName,DATA_COLLECTION_FOLDER_NAME)
            WINDOW_OUTPUT_TEXT += output_text
            window['-OUTPUT-'].update(WINDOW_OUTPUT_TEXT)

        #STOP AUDIO RECORDING EVENTS
        if event == STOP_AUDIO_RECORD_BUTTON:
            kill_running_processes_name("audio_recording",RUNNING_PROCESSES)
            time.sleep(0.5)
            WINDOW_OUTPUT_TEXT += "\nAUDIO RECORDING HAS BEEN COMPLETED TO FOLDER : {}/{}/{}".format(subject_id,sessionName,DATA_COLLECTION_FOLDER_NAME)
            window['-OUTPUT-'].update(WINDOW_OUTPUT_TEXT)

        #START ALLCAMERA RECORDING
        if event == START_RECORDING_DRIVING_BUTTON and not video_recorded_started:
            video_recorded_started = True
            check_counter += 1
            check_system_connection(window['-DEVICE-'], check_counter)
            if DATA_COLLECTION_FOLDER_NAME == '':
                DATA_COLLECTION_FOLDER_NAME = get_current_time()
            WINDOW_OUTPUT_TEXT += "\n\nSTARTING VISUAL/DRIVING RECORDING TO FOLDER : {}/{}/{}".format(subject_id,sessionName,DATA_COLLECTION_FOLDER_NAME)
            window['-OUTPUT-'].update(WINDOW_OUTPUT_TEXT)
            execute_bash_command_in_terminal(["bash",START_DRIVERS_SCRIPT],"camera_drivers")
            time.sleep(1)
            execute_bash_command_in_terminal(["bash", START_RECORDING_FOR_DRIVING_SCRIPT, DATA_COLLECTION_FOLDER_NAME,subject_id, sessionName] ,"all_camera_recording")
            check_recording_thread = threading.Thread(target=update_output_element, args=(WINDOW_OUTPUT_TEXT, CHECK_RECORDING_FOR_DRVING_SCRIPT, subject_id, sessionName,lambda : stop_check_status_thread), daemon=True)
            check_recording_thread.start() 

        #STOP ALLCAMERA RECORDING
        if event == STOP_RECORDING_DRIVING_BUTTON and video_recorded_started:
            stop_check_status_thread = True
            WINDOW_OUTPUT_TEXT += "\nVISUAL/DRIVING RECORDING HAS BEEN COMPLETED TO THE FOLDER : {}/{}/{}".format(subject_id,sessionName,DATA_COLLECTION_FOLDER_NAME)
            window['-OUTPUT-'].update(WINDOW_OUTPUT_TEXT)
            kill_running_processes_name("all_camera_recording",RUNNING_PROCESSES)
            time.sleep(0.5)
            kill_running_processes_name("camera_drivers",RUNNING_PROCESSES)
            video_recorded_started = False
            # check_recording_thread.join()
            

    kill_running_processes(RUNNING_PROCESSES)
    # Close the window and clean up
    window.close()

if __name__ == "__main__":
    try:

        argParser = argparse.ArgumentParser()
        argParser.add_argument("-s", "--subjectID", help="Subject ID",default="Subject_01")
        argParser.add_argument("-n", "--sessionName", help="Session Name",default="Baseline")
        args = argParser.parse_args()
        print("args=%s" % args)

        print("args.SubjectID=%s" % args.subjectID)
        print("args.SessionName=%s" % args.sessionName)
        main(args.subjectID, args.sessionName)
    except:
        kill_running_processes(RUNNING_PROCESSES)
        window.close()