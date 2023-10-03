# ""monitor.py -- Print all data received on a CAN channel

# This script uses canlib.canlib to listen on a channel and print all data
# received.

# It requires a CANlib channel with a connected device capable of receiving CAN
# messages and some source of CAN messages.

# The source of the messages may be e.g. the pinger.py example script.

# Also see the dbmonitor.py example script for how to look up the messages
# received in a database.

# """
import argparse
import shutil

from canlib import canlib
import pandas as pd
import os
import time
import datetime
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

REQUIRED_CAN_ID = [832,705,37,34,35]

def get_current_time():
    current_time = datetime.datetime.now()
    # return current_time.strftime('%d-%m-%y_%H:%M:%S')
    return current_time

def printframe(frame, width):
    form = '═^' + str(width - 1)
    print(format(" Frame received ", form))
    print("id:", frame.id)
    print("data:", bytes(frame.data))
    print("dlc:", frame.dlc)
    print("flags:", frame.flags)
    print("timestamp:", frame.timestamp)

def save_can_messages_to_csv(all_frames,save_path, start_timestamp):
            # Create a pandas DataFrame for the frames
   
    frame_data = {
        'id': [frame.id for frame in all_frames],
        'data': [frame.data for frame in all_frames],
        # 'dlc': [frame.dlc for frame in frames],
        # 'flags': [frame.flags for frame in frames],
        'timestamp': [frame.timestamp for frame in all_frames],
    }
    print("Total Rows: {}", len(all_frames))
    # Specify the maximum number of rows per CSV file
    max_rows_per_csv = 10000
    df = pd.DataFrame(frame_data)
    # Split the DataFrame into smaller DataFrames
    small_dfs = [df[i:i + max_rows_per_csv] for i in range(0, len(df), max_rows_per_csv)]

    # Create a directory to store the smaller CSV files
    # os.makedirs('/home/iac_user/output_csv_files', exist_ok=True)

    # Save each smaller DataFrame to a separate CSV file
    for i, small_df in enumerate(small_dfs):
        output_csv_file = f'/home/iac_user/output_csv_files/output_{i + 1}.csv'
        small_df.to_csv(output_csv_file, index=False)
        print("csv saved as :{}",output_csv_file)

def monitor_channel(channel_number, bitrate, ticktime,save_to_csv, save_path):
    ch = canlib.openChannel(channel_number, bitrate=bitrate)
    ch.setBusOutputControl(canlib.canDRIVER_NORMAL)
    ch.busOn()
    # required_can_ids = [832,705,37,34,35]
    width, height = shutil.get_terminal_size((80, 20))
    start_time = get_current_time()
    timeout = 0.5
    tick_countup = 0
    if ticktime <= 0:
        ticktime = None
    elif ticktime < timeout:
        timeout = ticktime
    all_frames = []
    print("Listening...")
    try:
        while True:
            try:
                
                frame = ch.read(timeout=int(timeout * 1000))
                if frame.id in REQUIRED_CAN_ID:
                    printframe(frame, width)
                    if save_to_csv:
                        all_frames.append(frame)
            except canlib.CanNoMsg:
                if ticktime is not None:
                    tick_countup += timeout
                    while tick_countup > ticktime:
                        print("tick")
                        tick_countup -= ticktime
            except KeyboardInterrupt:
                print("Stopping.")
                if save_to_csv:
                    if len(all_frames) > 5:
                        save_can_messages_to_csv(all_frames, save_path, start_time)
                ch.busOff()
                ch.close()
                break


    except Exception as e:
            print("Stopping.")
            if save_to_csv:
                if len(all_frames) > 5:
                    save_can_messages_to_csv(all_frames, save_path, start_time)
            ch.busOff()
            ch.close()    
    ch.busOff()
    ch.close()
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Listen on a CAN channel and print all frames received."
    )
    parser.add_argument('channel', type=int, default=0, nargs='?')
    # parser.add_argument(
    #     '--bitrate', '-b', default='500k', help=("Bitrate, one of " + ', '.join(bitrates.keys()))
    # )
    parser.add_argument(
        '--ticktime',
        '-t',
        type=float,
        default=0,
        help=("If greater than zero, display 'tick' every this many seconds"),
    )
    parser.add_argument(
        '--save',
        action="store_true",
        default=False
    )
    parser.add_argument(
        '-p',
        '--path',
        default='/home/iac_user/DATA_COLLECTION'
    )
    args = parser.parse_args()
    print(args.save)
    monitor_channel(args.channel, bitrates['500K'], args.ticktime, args.save, args.path)
