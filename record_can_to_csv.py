import pandas as pd
import argparse
import re
from canlib import canlib,Frame, kvadblib
import os
# Define a regular expression pattern to extract the desired information
pattern = r'id: (\d+)\ndata: b\'(.*?)\'\ndlc: (\d+)\nflags: (.+)\ntimestamp: (\d+)'
# pattern = re.compile(r"id:.*[0-9]+.*data:.*[^']*.*dlc:.*[0-9]+.*flags.*[A-Za-z]+\.[A-Za-z]+.*timestamp:.*[0-9]+", re.IGNORECASE)
decoded_frames = []
# Initialize empty lists to store the extracted data
frames = []
def convert(db):
    # Open the text file and extract data using regex
    db = kvadblib.Dbc(filename=db)
    with open('/home/iac_user/avalon_can_raw_output_0904.txt', 'r') as file:
        text = file.read()
        # print(text)
        matches = re.finditer(pattern, text)
        # num_matches = sum(1 for _ in matches)

        # # Reset the file pointer
        # file.seek(0)
        # print(len(matches[0][3]))
        # print("Number of Matches", num_matches)
        for match in matches:
            # print("here")
            try:
                frame_id = int(match.group(1))
                # print("Frame match: ",frame_id)
                data_hex = match.group(2)
                dlc = int(match.group(3))
                flags_str = match.group(4)
                timestamp = int(match.group(5))

                data_hex_clean = data_hex.replace('\\x', '')
                print(data_hex)
                data_values = [int(data_hex_clean[i:i + 2], 16) for i in range(0, len(data_hex_clean), 2)]
                
                # Map the flags string to MessageFlag enum
                flags_enum = getattr(canlib.MessageFlag, flags_str, canlib.MessageFlag.STD)

                # Create a Frame object and append it to the list
                frame = Frame(id_=frame_id, data=data_values, timestamp=timestamp, dlc=dlc, flags=flags_enum)
                try:
                    bmsg = db.interpret(frame)
                except kvadblib.KvdNoMessage:
                    print("<<< No message found for frame with id %s >>>" % frame.id)
                    continue
                    # decoded_frames.append(tuple())
                if not bmsg._message.dlc == bmsg._frame.dlc:
                    print(
                        "<<< Could not interpret message because DLC does not match for frame with id %s >>>"
                        % frame.id
                    )
                    print("\t- DLC (database): %s" % bmsg._message.dlc)
                    print("\t- DLC (received frame): %s" % bmsg._frame.dlc)
                    # decoded_frames.append(tuple())
                else:
                    decoded_msg = bmsg._message

                    print('ID: ',frame_id, '  Decoded Name: ', decoded_msg.name)

                    if decoded_msg.comment:
                        print(" Decoded Comment:", decoded_msg.comment)
                    signal_dict = {}
                    for bsig in bmsg:
                        signal_dict['name'] = bsig.name
                        signal_dict['value'] = bsig.value
                        signal_dict['unit'] = bsig.unit
                        # print('┃', bsig.name + ':', bsig.value, bsig.unit)
                    decoded_frames.append((decoded_msg.name, decoded_msg.comment, signal_dict))
                    frames.append(frame)
            except Exception as e:
                print(e)
    # Create a pandas DataFrame for the frames
    frame_data = {
        'id': [frame.id for frame in frames],
        'data': [frame.data for frame in frames],
        # 'dlc': [frame.dlc for frame in frames],
        # 'flags': [frame.flags for frame in frames],
        'timestamp': [frame.timestamp for frame in frames],
        'decoded_frame_name': [frame[0] for frame in decoded_frames],
        'decoded_signal': [frame[2] for frame in decoded_frames],
        'decoded_frame_comment':[frame[1] for frame in decoded_frames]
    }

    # Specify the maximum number of rows per CSV file
    max_rows_per_csv = 10000
    df = pd.DataFrame(frame_data)
    # Split the DataFrame into smaller DataFrames
    small_dfs = [df[i:i + max_rows_per_csv] for i in range(0, len(df), max_rows_per_csv)]

    # Create a directory to store the smaller CSV files
    os.makedirs('output_csv_files', exist_ok=True)

    # Save each smaller DataFrame to a separate CSV file
    for i, small_df in enumerate(small_dfs):
        output_csv_file = f'output_csv_files/output_{i + 1}.csv'
        small_df.to_csv(output_csv_file, index=False)
    # df = pd.DataFrame(frame_data)

    # # Save the DataFrame to a CSV file
    # df.to_csv('output.csv', index=False)

# Now you have a list of Frame objects and a DataFrame with the extracted data.
# You can access individual frames from the 'frames' list.
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Listen on a CAN channel and print all signals received, as specified by a database."
    )
    parser.add_argument(
        '--db',
        default="engine_example.dbc",
        help=("The database file to look up messages and signals in."),
    )
    args = parser.parse_args()

    convert(args.db)