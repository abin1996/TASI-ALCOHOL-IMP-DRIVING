import argparse
import pandas as pd
import os
import datetime
import time
from canlib import canlib, kvadblib

bitrates = {
    '1M': canlib.Bitrate.BITRATE_1M,
    '500K': canlib.Bitrate.BITRATE_500K,
    '250K': canlib.Bitrate.BITRATE_250K,
    '125K': canlib.Bitrate.BITRATE_125K,
    '100K': canlib.Bitrate.BITRATE_100K,
    '62K': canlib.Bitrate.BITRATE_62K,
    '50K': canlib.Bitrate.BITRATE_50K,
    '83K': canlib.Bitrate.BITRATE_83K,
    '10K': canlib.Bitrate.BITRATE_10K,
}
def get_current_time():
    current_time = datetime.datetime.now()
    return current_time.strftime('%d-%m-%y_%H-%M-%S')

def get_frame_data(db, frame,start_time, print_to_stdout=False):
    try:
        bmsg = db.interpret(frame)
    except kvadblib.KvdNoMessage:
        print("<<< No message found for frame with id %s >>>" % frame.id)
        print(frame.id)
        return None

    msg = bmsg._message
    if print_to_stdout:
        print('┏', msg.name)
        if msg.comment:
            print('┃', '"%s"' % msg.comment)
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
    for bsig in bmsg:
        cur_signals.append([bsig.name,bsig.value,bsig.unit])
    #cur message is a dict of the current message
    cur_message = {'time':cur_time,'frame_id':cur_frame_id,'msg_name':cur_msg_name,'signals':cur_signals}
    return cur_message

def save_frame_data_to_csv(filename, frame_data, last_updated):
    df = pd.DataFrame(frame_data)
    df.to_csv(filename,mode='a',header=not os.path.exists(filename),index=False)
    print("Saved {} messages to {} at {}".format(len(frame_data),filename, last_updated.strftime('%d-%m-%y_%H-%M-%S')))

def monitor_channel(channel_number, db_name, bitrate, filename):
    db = kvadblib.Dbc(filename=db_name)
    ch = canlib.openChannel(channel_number, canlib.canOPEN_ACCEPT_LARGE_DLC, bitrate=bitrate)
    ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
    # ch.readTimer()
    ch.busOn()

    timeout = 0.5
    all_frame_data = []
    print("Listening...")
    start_time = datetime.datetime.now()
    last_updated = start_time - datetime.timedelta(minutes=2)
    time_window = 60
    file_number = 0
    while True:

        try:
            frame = ch.read(timeout=int(timeout * 1000))
            if frame.id in [832,705,34,35,37]:
                frame_data = get_frame_data(db, frame, start_time)
                # print(last_updated)
                if frame_data is not None:
                    all_frame_data.append(frame_data)
                    #Update the CSV file every 60 seconds
                    if datetime.datetime.now() - last_updated > datetime.timedelta(seconds=time_window):
                        last_updated = datetime.datetime.now()
                        # sequenced_file_name = filename + '_{}'.formart(start_time.strftime('%d-%m-%y_%H-%M-%S')) +'_{}'.format(file_number)
                        save_frame_data_to_csv(filename, all_frame_data, last_updated)
                        all_frame_data = []
                        # file_number += 1
                        

        except canlib.CanNoMsg:
            # if ticktime is not None:
            print("No Message to Read")
        except KeyboardInterrupt:
            print("Stopping...")
            #Saving all_frame_data to a csv file using pandas dataframe
            # df = pd.DataFrame(all_frame_data)
            # df.to_csv(filename)
            ch.busOff()
            ch.close()
            break
        except Exception as e:
            print(e)
            print("Stopping...")
            #Saving all_frame_data to a csv file using pandas dataframe
            # df = pd.DataFrame(all_frame_data)
            # df.to_csv(filename)
            ch.busOff()
            ch.close()
            break

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Listen on a CAN channel and print all signals received, as specified by a database."
    )
    parser.add_argument(
        '--db',
        default="engine_example.dbc",
        help=("The database file to look up messages and signals in."),
    )
    parser.add_argument(
        '--bitrate', '-b', default='500k', help=("Bitrate, one of " + ', '.join(bitrates.keys()))
    )
    parser.add_argument(
        '--filename',
        '-f',
        type=str,
    )
    args = parser.parse_args()
    if args.filename is None:
        filename ='/home/iac_user/data_collection_scripts/can_{}.csv'.format(get_current_time())
    else :
        filename = args.filename

    monitor_channel(0, args.db, bitrates[args.bitrate.upper()], filename)