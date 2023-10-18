"""dbmonitor.py -- Read CAN messages using a database

This script will use canlib.canlib and canlib.kvadblib to monitor a CAN
channel, and look up all messages received in a database before printing them.

It requires a CANlib channel with a connected device capable of receiving CAN
messages, some source of CAN messages, and the same database that the source is
using to generate the messages.

In this example the channel is opened with flag canOPEN_ACCEPT_LARGE_DLC (optional).
This enables a DLC larger than 8 bytes (up to 15 for classic CAN). If 
canOPEN_ACCEPT_LARGE_DLC is excluded, generated frames with DLC > 8, will attain
a DLC of 8 on the receiving end, which may compromise the DLC equivalence 
check.

The source of the messages may be e.g. the pinger.py example script.

"""
import argparse
import pandas as pd
import time
import datetime
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

def get_frame_data(db, frame):
    try:
        bmsg = db.interpret(frame)
    except kvadblib.KvdNoMessage:
        print("<<< No message found for frame with id %s >>>" % frame.id)
        print(frame.id)
        return None

    # if not bmsg._message.dlc == bmsg._frame.dlc:
    #     print(
    #         "<<< Could not interpret message because DLC does not match for frame with id %s >>>"
    #         % frame.id
    #     )
    #     print("\t- DLC (database): %s" % bmsg._message.dlc)
    #     print("\t- DLC (received frame): %s" % bmsg._frame.dlc)
    #     return None

    msg = bmsg._message

    print('┏', msg.name)

    if msg.comment:
        print('┃', '"%s"' % msg.comment)

    for bsig in bmsg:
        print('┃', bsig.name + ':', bsig.value, bsig.unit)

    print('┗')

    cur_frame_id = frame.id
    cur_time = frame.timestamp
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

def monitor_channel(channel_number, db_name, bitrate, ticktime, filename):
    db = kvadblib.Dbc(filename=db_name)
    ch = canlib.openChannel(channel_number, canlib.canOPEN_ACCEPT_LARGE_DLC, bitrate=bitrate)
    ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
    ch.busOn()

    timeout = 0.5
    tick_countup = 0
    if ticktime <= 0:
        ticktime = None
    elif ticktime < timeout:
        timeout = ticktime
    all_frame_data = []
    print("Listening...")
    while True:
        try:
            frame = ch.read(timeout=int(timeout * 1000))
            # if frame.id in [832,705,34,35,37]:
            frame_data = get_frame_data(db, frame)
            if frame_data is not None:
                all_frame_data.append(frame_data)

        except canlib.CanNoMsg:
            if ticktime is not None:
                tick_countup += timeout
                while tick_countup > ticktime:
                    print("tick")
                    tick_countup -= ticktime
        except KeyboardInterrupt:
            print("Stopping")
            #Saving all_frame_data to a csv file using pandas dataframe
            
            df = pd.DataFrame(all_frame_data)
            df.to_csv(filename)
            ch.busOff()
            break


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Listen on a CAN channel and print all signals received, as specified by a database."
    )
    parser.add_argument(
        'channel', type=int, default=0, nargs='?', help=("The channel to listen on.")
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
        '--ticktime',
        '-t',
        type=float,
        default=0,
        help=("If greater than zero, display 'tick' every this many seconds"),
    )
    args = parser.parse_args()
    filename ='/home/iac_user/data_collection_scripts/can_bus_output/can_{}.csv'.format(get_current_time())

    monitor_channel(args.channel, args.db, bitrates[args.bitrate.upper()], args.ticktime,filename)