#send_msg
# The CANlib library is initialized when the canlib module is imported. To be
# able to send a message, Frame also needs to be installed.
from canlib import canlib, Frame, kvadblib
import pandas as pd
import datetime
import time
import argparse
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
    cur_raw_data = frame.data 
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
    
    cur_message = {'time':cur_time,'frame_id':cur_frame_id,'msg_name':cur_msg_name,'signals':cur_signals, 'result_ready':result_ready, 'test_count':test_count, 'test_failed':test_failed, 'test_status':test_status, 'raw_data':cur_raw_data}

    return cur_message

def save_to_csv(filename, all_frame_data):
    df = pd.DataFrame(all_frame_data)
    df.to_csv(filename,columns=['time','frame_id','msg_name','signals', 'raw_data'])


def record_brac(filename, start_time,db, window_output, subject_id):
    output_text = "Starting BRAC test for Subject {}".format(subject_id)
    db = kvadblib.Dbc(filename='/home/iac_user/data_collection_scripts/dadss_breath_sensor_fixed.dbc')
    try:
        ch_a = canlib.openChannel(channel=1)
        ch_a.setBusParams(canlib.canBITRATE_500K)
        ch_a.busOn()

        window_output.update(output_text)
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
        while True:
            try:
                msg = ch_a.read(timeout=-1)

                if msg.id == 783:
                    decoded_frame = get_frame_data(db, msg, start_time, print_to_stdout=print_to_stdout)
                    all_frame_data.append(decoded_frame)
                    if latest_estatus != decoded_frame['test_status']:
                        if decoded_frame['test_status'] == 2:
                            output_text += "\nSensor Starting now. Please Wait..."
                            window_output.update(output_text)
                            frame = Frame(id_=800, data=bytearray(b'\x01\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
                            ch_a.write(frame)
                            time.sleep(0.1)
                            frame = Frame(id_=800, data=bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
                            ch_a.write(frame)
                        if decoded_frame['test_status'] == 4 and not break_loop:
                            output_text += "\nBlow into the sensor now..."
                            window_output.update(output_text)
                        else:
                            output_text += "\nCurrent Sensor Status: {}".format(decoded_frame['test_status'])
                            window_output.update(output_text)
                        latest_estatus = decoded_frame['test_status']
                    
                    if decoded_frame['test_count'] != num_of_tests_done:
                        num_of_tests_done = decoded_frame['test_count']
                        # window_output.update("Total Tests Done: {}".format(num_of_tests_done))   
                    
                    if decoded_frame['test_failed'] and decoded_frame['test_count'] not in failed_tests:
                        failed_tests.add(decoded_frame['test_count'])
                        number_of_failed_tests += 1
                        output_text += "\nTEST FAILED"
                        window_output.update(output_text)
                        # window_output.update("Number of Failed Tests: {}".format(number_of_failed_tests))

                    if decoded_frame['result_ready'] and decoded_frame['test_count'] not in success_tests:
                        success_tests.add(decoded_frame['test_count'])
                        number_of_successful_tests += 1
                        
                        # window_output.update("Number of Succesful Tests Done: {}".format(number_of_successful_tests))

                        if number_of_successful_tests == 1:
                            output_text += "\nTEST 1 SUCCESSFUL"
                            window_output.update(output_text)
                            # window_output.update("Blow into the sensor after 1 second...")
                        if number_of_successful_tests == 2:
                            output_text += "\nTEST 2 SUCCESSFUL"
                            window_output.update(output_text)
                            output_text += "\nClosing the BRAC sensor and saving results"
                            window_output.update(output_text)

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
                    output_text += "\nWaiting for message"
                    window_output.update(output_text)
            except KeyboardInterrupt:
                window_output.update("A keyboard Interuption Occured. Closing the BRAC sensor")
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
    except canlib.canError:
        window_output.update("Error in opening the CAN channel. Please check the connection and try again")
        return


# if __name__ == "__main__":
#     start_time = datetime.datetime.now()
#     filename ='/home/iac_user/data_collection_scripts/brac_test/brac_{}.csv'.format(start_time.strftime('%d-%m-%y_%H-%M-%S'))
#     record_brac(filename, start_time)

# if __name__ == '__main__':
#     start_time = datetime.datetime.now()
#     parser = argparse.ArgumentParser(
#         description="Listen on a CAN channel and print all signals received, as specified by a database."
#     )
#     parser.add_argument(
#         '--db',
#         default="engine_example.dbc",
#         help=("The database file to look up messages and signals in."),
#     )
#     parser.add_argument(
#         '--filename',
#         '-f',
#         type=str,
#     )
#     args = parser.parse_args()
#     if args.filename is None:
#         filename ='/home/iac_user/data_collection_scripts/brac_test/brac_{}.csv'.format(start_time.strftime('%d-%m-%y_%H-%M-%S'))
#     else :
#         filename = args.filename

#     if args.db is None:
#         db = kvadblib.Dbc(filename='/home/iac_user/data_collection_scripts/dadss_breath_sensor_fixed.dbc')
#     else :
#         db = args.db
#     record_brac(filename, start_time, db,None) 