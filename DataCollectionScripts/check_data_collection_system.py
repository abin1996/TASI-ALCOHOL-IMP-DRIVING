# check_ch
# Firstly import canlib so that it can be used in the script.
from canlib import canlib
import subprocess

def check_can_bus(output_text=None):
    # .getNumberOfChannels() is used to detect the number of channels and
    # the number is saved in the variable num_channels.
    num_channels = canlib.getNumberOfChannels()
    ret_code = 0
    if num_channels == 0:
        output_text += "\nERROR: No CAN channels found. Reinstall CAN drivers using fix_can_bus.sh"
        ret_code = 1
    if num_channels == 2:
        output_text += "\nERROR: CAN Device not connected."
        ret_code = 1
    if num_channels >2:
        output_text += "\nCAN: ALL GOOD            -- CAN Devices Connected"
        ret_code = 0
    return ret_code, output_text

def check_microphone(output_text=None):
    #run the os command "arecord --list-devices | grep H650e" and if there is no output(output is empty) then raise an error
    #else print "MICROPHONE:ALL GOOD"
    output = subprocess.check_output("arecord --list-devices", shell=True)
    ret_code = 0
    if b'H650e' not in output:
        output_text += "\nERROR: Microphone not connected."
        ret_code = 1
    else:
        output_text += "\nMICROPHONE: ALL GOOD   -- Microphone connected"
        ret_code = 0
    return ret_code, output_text

def check_cameras(output_text=None):
    #run the os command "ls /dev/video*" and if there is no output(output is empty) then raise an error
    #else print "CAMERAS:ALL GOOD"
    output = subprocess.check_output("ls /dev/v4l/by-id/*", shell=True)
    ret_code = 0
    cams_connected = 0
    if b'113DD05F' in output:
        cams_connected += 1
    if b'926BD05F' in output:
        cams_connected += 1
    if b'9DA5762F' in output:
        cams_connected += 1
    if b'BD45762F' in output:
        cams_connected += 1
    if cams_connected == 4:
        output_text += "\nCAMERAS: ALL GOOD      -- 4 Cameras connected"
        ret_code = 0
    else:
        output_text += "\nERROR: {} Cameras not connected.".format(4-cams_connected)
        ret_code = 1
    return ret_code, output_text

def check_gps(output_text=None):
    #The GPS device can be verified by checking for the IP 192.168.1.6 and seeing if that IP is connected to the system
    #If the IP is not connected then raise an error
    #else print "GPS:ALL GOOD"
    output = subprocess.check_output("arp -a 192.168.1.6", shell=True)
    ret_code = 0
    if b'c0:84:7d:1e:c8:5a' in output:
        output_text += "\nGPS: ALL GOOD          -- GPS Device connected"
        ret_code = 0
    else:
        output_text += "\nERROR: GPS Device not connected."
        ret_code = 1
    
    return ret_code, output_text

def check_system_connection(window=None, check_counter=0):
    ret_code = 0
    output_text = str()
    ret_code_cameras, output_text = check_cameras(output_text)
    ret_code_gps, output_text = check_gps(output_text)
    ret_code_can, output_text = check_can_bus(output_text)
    ret_code_microphone, output_text = check_microphone(output_text)
    if ret_code_can == 1 or ret_code_microphone == 1 or ret_code_cameras == 1 or ret_code_gps == 1:
        ret_code = 1
    
    if window is not None:
        output_text += "\nChecked {} times".format(check_counter)
        window.update(output_text)
    else:
        print(output_text)
    return ret_code

if __name__ == "__main__":
    check_system_connection()

