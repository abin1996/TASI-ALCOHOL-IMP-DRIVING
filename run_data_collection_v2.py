import subprocess
import platform
import time
import os
import signal
import PySimpleGUI as sg
import threading
import re
import datetime
import argparse

DATA_COLLECTION_FOLDER_NAME = ''

START_RECORD_AUDIO_BUTTON = "AUDIO TEST: START RECORD"
STOP_AUDIO_RECORD_BUTTON = "AUDIO TEST: STOP RECORD"

START_BRAC_BUTTON = "BREATH SENSOR: START RECORD"

START_RECORDING_DRIVING_BUTTON = "EYE TRACKING AND DRIVING TEST: START RECORD"
STOP_RECORDING_DRIVING_BUTTON = "EYE TRACKING AND DRIVING TEST: STOP RECORD"


ROSCORE_START_SCRIPT = "roscore"
START_DRIVERS_SCRIPT = "/home/iac_user/start_drivers.sh"
START_RECORDING_FOR_DRIVING_SCRIPT = "/home/iac_user/data_collection_scripts/start_recording_cam_gps_joy.sh"
CHECK_RECORDING_FOR_DRVING_SCRIPT = "/home/iac_user/data_collection_scripts/check_recording_v2.sh"
START_AUDIO_RECORDING_SCRIPT = "/home/iac_user/data_collection_scripts/start_audio_recording.sh"

WINDOW_OUTPUT_TEXT= "*"
# Define the layout of the GUI
layout = [
    [sg.Button(START_BRAC_BUTTON,button_color="green")],
    # [sg.Button(STOP_BRAC_BUTTON,button_color="red")],
    [sg.Button(START_RECORD_AUDIO_BUTTON,button_color="green")],
    [sg.Button(STOP_AUDIO_RECORD_BUTTON,button_color="red")],
    [sg.Button(START_RECORDING_DRIVING_BUTTON,button_color="green")],
    [sg.Button(STOP_RECORDING_DRIVING_BUTTON,button_color="red")],
    [sg.Frame("Output", [[sg.Multiline(disabled=True,size=(150, 30), font=("Courier New", 25), key="-OUTPUT-", background_color="black", text_color="white")]],expand_x=True,expand_y=True)],
    [sg.Button("Exit")]
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
    if platform.system() == "Windows":
        terminal_command = f'start cmd /c "{command}"'
    elif platform.system() == "Linux":
        terminal_command = f'gnome-terminal -- {command}'
    elif platform.system() == "Darwin":
        terminal_command = f'osascript -e \'tell application "Terminal" to do script "{command}"\''
    else:
        raise ValueError("Unsupported platform.")
    print(command)
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

def call_bash_script(script_path, window_text, subject_id):
    try:
        # Run the Bash script and capture the output
        window['-OUTPUT-'].update(window_text)
        # print("-------- Recording Status --------")
        command = ["bash", script_path, subject_id]
        print(command)
        result = subprocess.run(["bash", script_path, subject_id], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
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
    rec_file_name = get_rec_file_name(script_resp)
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
        bag_number = data["bag_number"]
        if status.startswith("ERROR"):
            display_text += "\n"+"!!!!!!!!!!  ERROR  !!!!!!!!!!" 

        display_text += "\n" + f"{category} -- Bag Number:{bag_number}, Status: {status}"
    window_text += display_text
    window['-OUTPUT-'].update(window_text)

def update_output_element(window_text, script_path, subject_id):
    while True:
        time.sleep(30)
        call_bash_script(script_path,window_text, subject_id)
        # Adjust the update interval as needed

def call_and_wait_for_breath_sensor(DATA_COLLECTION_FOLDER_NAME,subject_id):
    pass

def main(subject_id):
    # Event loop to process events and get button clicks
    global DATA_COLLECTION_FOLDER_NAME
    global RUNNING_PROCESSES
    global WINDOW_OUTPUT_TEXT
    execute_bash_command_in_terminal([ROSCORE_START_SCRIPT],"roscore")
    time.sleep(0.5)
    while True:
        event, values = window.read()
        if event in (sg.WINDOW_CLOSED, "Exit"):
            break
            
        #BREATH SENSOR EVENTS
        if event == START_BRAC_BUTTON:
            if DATA_COLLECTION_FOLDER_NAME == '':
                DATA_COLLECTION_FOLDER_NAME = get_current_time()
            execute_bash_command_in_terminal(["bash", START_BRAC_BUTTON, DATA_COLLECTION_FOLDER_NAME,subject_id],"breath_sensor")


        #AUDIO RECORDING EVENTS
        if event == START_RECORD_AUDIO_BUTTON:
            if DATA_COLLECTION_FOLDER_NAME == '':
                DATA_COLLECTION_FOLDER_NAME = get_current_time()
            execute_bash_command_in_terminal(['bash', START_AUDIO_RECORDING_SCRIPT, DATA_COLLECTION_FOLDER_NAME, subject_id],"audio_recording",is_shell=False)
            output_text = "\nSTARTED AUDIO RECORDING IN FOLDER: {}/{}".format(subject_id,DATA_COLLECTION_FOLDER_NAME)
            WINDOW_OUTPUT_TEXT += output_text
            window['-OUTPUT-'].update(output_text)
        if event == STOP_AUDIO_RECORD_BUTTON:
            kill_running_processes_name("audio_recording",RUNNING_PROCESSES)
            time.sleep(0.5)
            WINDOW_OUTPUT_TEXT += "\nAUDIO RECORDING HAS BEEN COMPLETED TO FOLDER : {}/{}".format(subject_id,DATA_COLLECTION_FOLDER_NAME)
            window['-OUTPUT-'].update(WINDOW_OUTPUT_TEXT)

        #ALLCAMERA RECORDING
        if event == START_RECORDING_DRIVING_BUTTON:
            if DATA_COLLECTION_FOLDER_NAME == '':
                DATA_COLLECTION_FOLDER_NAME = get_current_time()
            WINDOW_OUTPUT_TEXT += "\nSTARTING VISUAL/DRIVING RECORDING TO FOLDER : {}/{}".format(subject_id,DATA_COLLECTION_FOLDER_NAME)
            window['-OUTPUT-'].update(WINDOW_OUTPUT_TEXT)
            execute_bash_command_in_terminal(["bash",START_DRIVERS_SCRIPT],"camera_drivers")
            time.sleep(1)
            execute_bash_command_in_terminal(["bash", START_RECORDING_FOR_DRIVING_SCRIPT, DATA_COLLECTION_FOLDER_NAME,subject_id] ,"all_camera_recording")
            output_thread = threading.Thread(target=update_output_element, args=(WINDOW_OUTPUT_TEXT, CHECK_RECORDING_FOR_DRVING_SCRIPT, subject_id), daemon=True)
            output_thread.start()    
        if event == STOP_RECORDING_DRIVING_BUTTON:
            WINDOW_OUTPUT_TEXT += "\nVISUAL/DRIVING RECORDING COMPLETED"
            window['-OUTPUT-'].update(WINDOW_OUTPUT_TEXT)
            kill_running_processes_name("all_camera_recording",RUNNING_PROCESSES)
            time.sleep(0.5)
            kill_running_processes_name("camera_drivers",RUNNING_PROCESSES)
            time.sleep(0.5)
            break

    kill_running_processes(RUNNING_PROCESSES)
    # Close the window and clean up
    window.close()

if __name__ == "__main__":
    try:

        argParser = argparse.ArgumentParser()
        argParser.add_argument("-s", "--subjectID", help="Subject ID",default="Subject_01")

        args = argParser.parse_args()
        print("args=%s" % args)

        print("args.SubjectID=%s" % args.subjectID)

        main(args.subjectID)
    except:
        kill_running_processes(RUNNING_PROCESSES)
        window.close()