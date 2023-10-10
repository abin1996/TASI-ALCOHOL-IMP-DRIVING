#send_msg
# The CANlib library is initialized when the canlib module is imported. To be
# able to send a message, Frame also needs to be installed.
from canlib import canlib, Frame, kvadblib
import pandas as pd
import datetime
import time
RESULTS = []

def get_frame_data(db, frame, start_time, print_to_stdout=False):
    try:
        bmsg = db.interpret(frame)
    except kvadblib.KvdNoMessage:
        print("<<< No message found for frame with id %s >>>" % frame.id)
        print(frame.id)
        return

    if not bmsg._message.dlc == bmsg._frame.dlc:
        print(
            "<<< Could not interpret message because DLC does not match for frame with id %s >>>"
            % frame.id
        )
        print("\t- DLC (database): %s" % bmsg._message.dlc)
        print("\t- DLC (received frame): %s" % bmsg._frame.dlc)
        return

    msg = bmsg._message
    if print_to_stdout:
        print('┏', msg.name)

        if msg.comment:
            print('┃', '"%s"' % msg.comment)
        # print(bmsg)
        for bsig in bmsg:
            print('┃', bsig.name + ':', bsig.value, bsig.unit)

        print('┗')
    cur_frame_id = frame.id
    cur_time = start_time + datetime.timedelta(milliseconds=float(frame.timestamp))
    cur_time = cur_time.strftime('%d-%m-%y_%H:%M:%S.%f')
    cur_data = frame.data 
    cur_dlc = frame.dlc
    cur_flags = frame.flags
    cur_msg_name = msg.name
    cur_msg_comment = msg.comment
    cur_signals = []
    result_ready = False 
    test_failed = False
    test_count = None
    test_status = None
    for bsig in bmsg:
        cur_signals.append([bsig.name,bsig.value,bsig.unit])
        if bsig.name == 'e_Status':
            test_status = bsig.value
        if bsig.name == 'e_Status' and bsig.value == 6:
            result_ready = True
        if bsig.name == 'e_Status' and bsig.value == 5:
            test_failed = True
        if bsig.name == 'n_BrAC_Count':
            test_count = bsig.value
    
    cur_message = {'time':cur_time,'frame_id':cur_frame_id,'msg_name':cur_msg_name,'signals':cur_signals, 'result_ready':result_ready, 'test_count':test_count, 'test_failed':test_failed, 'test_status':test_status}

    return cur_message

def save_to_csv(filename, all_frame_data):
    df = pd.DataFrame(all_frame_data)
    df.to_csv(filename,columns=['time','frame_id','msg_name','signals'])


def record_brac(filename, start_time):
    db = kvadblib.Dbc(filename='/home/iac_user/data_collection_scripts/dadss_breath_sensor_fixed.dbc')
    ch_a = canlib.openChannel(channel=1)
    ch_a.setBusParams(canlib.canBITRATE_500K)
    ch_a.busOn()
    print("Starting BRAC test")
    frame = Frame(id_=800, data=bytearray(b'\x01\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
    ch_a.write(frame)
    time.sleep(0.1)
    frame = Frame(id_=800, data=bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
    ch_a.write(frame)

    all_frame_data = []
    
    failed_tests = set()
    success_tests = set()
    break_loop = False
    channel_closed = False
    print_to_stdout = False

    num_of_tests_done = 0
    latest_estatus = 0
    number_of_failed_tests = 0
    number_of_successful_tests = 0
    print("Blow into the sensor")
    while True:
        try:
            msg = ch_a.read(timeout=-1)

            if msg.id == 783:
                decoded_frame = get_frame_data(db, msg, start_time, print_to_stdout=print_to_stdout)
                all_frame_data.append(decoded_frame)
                if latest_estatus != decoded_frame['test_status']:
                    print("Status: {}".format(decoded_frame['test_status']))
                    latest_estatus = decoded_frame['test_status']
                
                if decoded_frame['test_count'] != num_of_tests_done:
                    num_of_tests_done = decoded_frame['test_count']
                    print("Total Tests Done: {}".format(num_of_tests_done))   
                
                if decoded_frame['test_failed'] and decoded_frame['test_count'] not in failed_tests:
                    failed_tests.add(decoded_frame['test_count'])
                    number_of_failed_tests += 1
                    print("Test FAILED")
                    print("Number of Failed Tests: {}".format(number_of_failed_tests))

                if decoded_frame['result_ready'] and decoded_frame['test_count'] not in success_tests:
                    success_tests.add(decoded_frame['test_count'])
                    number_of_successful_tests += 1
                    print("Number of Succesful Tests Done: {}".format(number_of_successful_tests))
                    frame = Frame(id_=800, data=bytearray(b'\x01\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
                    ch_a.write(frame)
                    time.sleep(0.1)
                    frame = Frame(id_=800, data=bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
                    ch_a.write(frame)
                    if number_of_successful_tests == 2:
                        break_loop = True

            if msg.id == 799:
                decoded_frame = get_frame_data(db, msg, start_time, print_to_stdout=print_to_stdout)
                all_frame_data.append(decoded_frame)
                if number_of_successful_tests == 2:
                        if break_loop:
                            break

        except canlib.CanNoMsg:
                # if ticktime is not None:
                print("Waiting for message")
        except KeyboardInterrupt:
            print("Stopping")
            frame = Frame(id_=800, data=bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
            ch_a.write(frame)
            ch_a.busOff()
            ch_a.close()
            channel_closed = True
            save_to_csv(filename, all_frame_data)
            break

    save_to_csv(filename, all_frame_data)
    if not channel_closed:
        frame = Frame(id_=800, data=bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
        ch_a.write(frame)
        ch_a.busOff()
        ch_a.close()

if __name__ == "__main__":
    start_time = datetime.datetime.now()
    filename ='/home/iac_user/data_collection_scripts/brac_test/brac_{}.csv'.format(start_time.strftime('%d-%m-%y_%H-%M-%S'))
    record_brac(filename)
