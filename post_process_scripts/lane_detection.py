import cv2
import os
import matplotlib.pyplot as plt
import re

import csv
from datetime import datetime

num_trans_hor_pixel = 1450
num_trans_ver_pixel = 750
mat_hor_len = 1455 # unit: mm
dpp_hor = mat_hor_len / num_trans_hor_pixel

veh_width = 1830 #mm
veh2mat = 100 #mm

# Function to extract timestamp from a filename
def extract_timestamp(filename):
    match = re.search(r"bag_\d+_frame_\d+_(\d+)", filename)    
    if match:
        return int(match.group(1))
    return None

# Function to convert timestamp to datetime object
def timestamp_to_datetime(timestamp):
    return datetime.fromtimestamp(timestamp / 1e9)  # Convert nanoseconds to seconds

def distance2lane(image_folder, output_csv_folder_path, image_folder_name):
    if not os.path.exists(output_csv_folder_path):
        os.makedirs(output_csv_folder_path, exist_ok=True)
    csv_filename = image_folder_name + '_distance_to_lane.csv'
    if os.path.exists(os.path.join(output_csv_folder_path, csv_filename)):
        os.remove(os.path.join(output_csv_folder_path, csv_filename))
    # Get the list of PNG files in the folder
    png_files = [filename for filename in os.listdir(image_folder) if filename.endswith(".png") and filename.startswith("bag")]

    # Sort the PNG files based on the extracted timestamps
    sorted_png_files = sorted(png_files, key=lambda filename: extract_timestamp(filename))

    # Change the working directory to the new directory
    os.chdir(image_folder)

    # Create a list of dictionaries to store data
    data_list = []

    # Iterate through each image in the folder
    for filename in sorted_png_files:

        full_path = os.path.join(image_folder, filename)

        if filename.endswith(('.jpg', '.png', '.jpeg')):  # Ensure you only process image files

            # image_path = os.path.join(image_folder, filename)
            # Load the image
            image = cv2.imread(full_path)

            # Extract the timestamp from the filename
            timestamp = extract_timestamp(filename)
            # Convert the timestamp to a datetime object
            datetime_obj = timestamp_to_datetime(timestamp)

            # print('Imgage:',full_path)
            # print('Timestamp:', timestamp)

            # 1. Convert the RGB image to grayscale
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # 2. Define the vertical position range
            start_vertical = 450
            end_vertical = 550

            # 3. Create an empty list to store the gray values
            grey_values = []
            horizontal_pos = []
            # 4. Iterate through horizontal positions and calculate gray value sums
            for horizontal in range(0, 1450):
                gray_sum = sum(gray_image[start_vertical:end_vertical, horizontal])
                grey_values.append(gray_sum)
                horizontal_pos.append(horizontal)

            # grey_values now contains the sums of grayscale values for each horizontal position in the specified vertical range.
            # print(grey_values)

            # # Create the plot
            plt.plot(horizontal_pos, grey_values)
            plt.xlabel('Horizontal Position')
            plt.ylabel('Grey Value')
            plt.title('Grey Value Plot')
            # Show the plot
            # plt.show()
            # plt.pause(1)

            # if grey_values:
            largest_value = max(grey_values)

            # Calculate the threshold value (80% of the largest value)
            threshold_value_1 = 0.85 * largest_value
            threshold_value_2 = 0.75 * largest_value

            # Find the corresponding x position when the gray value is between 75% and 85% of the largest value
            indices = [i for i, x in enumerate(grey_values) if threshold_value_2 <= x <= threshold_value_1]

                # if indices:
            detected_lane_width = ((max(indices) - min(indices))*dpp_hor)/10

            # print('max ind:', max(indices))
            # print('min ind:', min(indices))

            print('Detected lane width is',round(detected_lane_width,2), 'cm')

            cam_to_lane_center  = (max(indices) + min(indices))/2 *dpp_hor/10

            veh_to_lane_center = cam_to_lane_center + veh_width/20 + 10

            print('Vehicle to boundary lane is',round(veh_to_lane_center,2) , 'cm')
            print('---------------')

            # Create a dictionary to store data for this image
            data_dict = {
                'Date time': datetime_obj,
                'Timestamp':timestamp,
                'Distance_to_Lane (cm)': veh_to_lane_center
            }

            data_list.append(data_dict)

    
    # Save the data as a CSV file
    csv_file_path = os.path.join(output_csv_folder_path, csv_filename)

    with open(csv_file_path, mode='w', newline='') as csv_file:
        fieldnames = ['Date time', 'Timestamp','Distance_to_Lane (cm)']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        for data_dict in data_list:
            writer.writerow(data_dict)

    print('CSV file saved:', csv_file_path)
    return data_list

if __name__ == "__main__":

    # transformed right images
    # new_directory = "D:/HZTs_file/Impaired-Driving/test_images/Baseline/Driving backward/Driving backward_1/images/transformed_image_right"  # Replace with the path to your new directory
    # Partial transfored right images
    # new_directory = "D:/HZTs_file/Impaired-Driving/test_images/Baseline/Driving backward/Driving backward_1/images/transformed_image_right_test"

    # transformed left images
    # new_directory = "D:/HZTs_file/Impaired-Driving/test_images/Baseline/Driving backward/Driving backward_1/images/transformed_image_left_flipped"  # Replace with the path to your new directory

    # Partial transformed left images
    new_directory = "D:/HZTs_file/Impaired-Driving/test_images/Baseline/Driving backward/Driving backward_1/images/transformed_image_left_flipped_test"  # Replace with the path to your new directory

    # test images
    # new_directory = "D:/HZTs_file/Impaired-Driving/test_img/images3_test/transformed_images3_test"
    '''''''''
    The input_folder of the distance2lane function is the folder that contains the transformed frames
    '''''''''
    #TODO: Current function generates a csv that contains the distance to one side. May need to give two input folders (left and right) and generate the distances in one csv file
    distance2lane(new_directory)