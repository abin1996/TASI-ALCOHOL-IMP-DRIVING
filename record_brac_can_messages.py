#send_msg
# The CANlib library is initialized when the canlib module is imported. To be
# able to send a message, Frame also needs to be installed.
from canlib import canlib, Frame, kvadblib
import pandas as pd
import datetime
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
    for bsig in bmsg:
        cur_signals.append([bsig.name,bsig.value,bsig.unit])
        if bsig.name == 'e_Status' and bsig.value == 6:
            result_ready = True
        if bsig.name == 'e_Status' and bsig.value == 5:
            test_failed = True
        if bsig.name == 'n_BrAC_Count':
            test_count = bsig.value
    
    cur_message = {'time':cur_time,'frame_id':cur_frame_id,'msg_name':cur_msg_name,'signals':cur_signals, 'result_ready':result_ready, 'test_count':test_count, 'test_failed':test_failed}

    return cur_message



def record_brac(filename):
    db = kvadblib.Dbc(filename='/home/iac_user/data_collection_scripts/dadss_breath_sensor_fixed.dbc')
    ch_a = canlib.openChannel(channel=1)
    ch_a.setBusParams(canlib.canBITRATE_500K)
    ch_a.busOn()
    print("Starting BRAC test")
    frame = Frame(id_=800, data=bytearray(b'\x01\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
    ch_a.write(frame)

    all_frame_data = []
    start_time = datetime.datetime.now()
    latest_brac_count = None
    print("Blow into the sensor")
    num_of_results = 0
    failed_tests = set()
    break_loop = False
    channel_closed = False
    while True:
        try:
            msg = ch_a.read(timeout=500)
            if msg.id == 783:
                # print("Status:")
                # print(msg)
                decoded_frame = get_frame_data(db, msg, start_time, print_to_stdout=False)
                all_frame_data.append(decoded_frame)
                if not latest_brac_count:
                    print("setting latest_brac_count {}".format(decoded_frame['test_count']))
                    latest_brac_count = decoded_frame['test_count']
                
                if decoded_frame['test_failed'] and decoded_frame['test_count'] not in failed_tests:
                    failed_tests.add(decoded_frame['test_count'])
                    if num_of_results > 0:
                        num_of_results -= 1
                    print("Test FAILED")
                    print("Tests Done: {}".format(num_of_results))


                elif decoded_frame['test_count'] > latest_brac_count:
                    num_of_results += 1
                    print("Tests Done: {}".format(num_of_results))
                    latest_brac_count = decoded_frame['test_count']
                    if num_of_results == 2:
                        break_loop = True

                if decoded_frame['result_ready']:
                    num_of_results += 1
                    print("Tests Done: {}".format(num_of_results))
                    latest_brac_count = decoded_frame['test_count']
                    if num_of_results == 2:
                        break_loop = True

                
                # break
            if msg.id == 799:
                # print("Results:")
                # print(msg)
                decoded_frame = get_frame_data(db, msg, start_time, print_to_stdout=False)
                all_frame_data.append(decoded_frame)
                if num_of_results == 2:
                        if break_loop:
                            break
                    # printframe(db, msg)
                    # break
                # break
        except canlib.CanNoMsg:
                # if ticktime is not None:
                print("No Message to Read")
        except KeyboardInterrupt:
            print("Stopping")
            frame = Frame(id_=800, data=bytearray(b'\x04\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
            ch_a.write(frame)
            ch_a.busOff()
            ch_a.close()
            channel_closed = True
            df = pd.DataFrame(all_frame_data)
            df.to_csv(filename)
            break
    df = pd.DataFrame(all_frame_data)
    df.to_csv(filename)
        # except Exception as e:
        #     print("Stopping")
        #     ch_a.busOff()
        #     ch_a.close()
        #     break
            #Saving all_frame_data to a csv file using pandas dataframe
            # df = pd.DataFrame(all_frame_data)
            # df.to_csv(filename)
    if not channel_closed:
        frame = Frame(id_=800, data=bytearray(b'\x04\x00\x00\x00\x00\x00\x00\x00'), flags=canlib.MessageFlag.STD)
        ch_a.write(frame)
        ch_a.busOff()
        ch_a.close()

if __name__ == "__main__":
    filename ='/home/iac_user/data_collection_scripts/brac_test.csv'
    record_brac(filename)
