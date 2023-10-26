# check_ch
# Firstly import canlib so that it can be used in the script.
from canlib import canlib

def check_can_bus():
    # .getNumberOfChannels() is used to detect the number of channels and
    # the number is saved in the variable num_channels.
    num_channels = canlib.getNumberOfChannels()
    if num_channels == 0:
        print("ERROR: No CAN channels found. Reinstall CAN drivers using fix_can_bus.sh")
        return 1
    if num_channels == 2:
        print("ERROR: CAN Device not connected. Connect CAN Device and try again.")
        return 1
    if num_channels >2:
        print("CAN: ALL GOOD            -- CAN Devices Connected")
        return 0

def check_microphone():
    #run the os command "arecord --list-devices | grep H650e" and if there is no output(output is empty) then raise an error
    #else print "MICROPHONE:ALL GOOD"
    import subprocess
    output = subprocess.check_output("arecord --list-devices", shell=True)
    if b'H650e' not in output:
        print("ERROR: Microphone not connected. Connect Microphone and try again.")
        return 1
    else:
        print("MICROPHONE: ALL GOOD     -- 1 Microphone connected")
        return 0

def check_cameras():
    #run the os command "ls /dev/video*" and if there is no output(output is empty) then raise an error
    #else print "CAMERAS:ALL GOOD"
    import subprocess
    output = subprocess.check_output("ls /dev/v4l/by-id/*", shell=True)
    if b'113DD05F' in output  and b'926BD05F' in output and b'9DA5762F' in output and b'BD45762F' in output:
        print("CAMERAS: ALL GOOD        -- 4 Cameras connected")
        return 0
    else:
        print("ERROR: Cameras not connected. Connect Cameras and try again.")
        return 1

def check_gps():
    #The GPS device can be verified by checking for the IP 192.168.1.6 and seeing if that IP is connected to the system
    #If the IP is not connected then raise an error
    #else print "GPS:ALL GOOD"
    import subprocess
    output = subprocess.check_output("arp -a 192.168.1.6", shell=True)
    if b'c0:84:7d:1e:c8:5a' in output:
        print("GPS: ALL GOOD            -- GPS connected at IP:192.168.1.6")
        return 0
    else:
        print("ERROR: GPS not connected. Wait for GPS to connect to Hotspot and try again.")
        return 1

def check_system_connection():
    ret_code = 0
    ret_code_can = check_can_bus()
    ret_code_microphone = check_microphone()
    ret_code_cameras = check_cameras()
    ret_code_gps = check_gps()
    if ret_code_can == 1 or ret_code_microphone == 1 or ret_code_cameras == 1 or ret_code_gps == 1:
        ret_code = 1
    return ret_code
if __name__ == "__main__":
    check_system_connection()

