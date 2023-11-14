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

def distance2lane(image_folder, output_csv_folder_path, image_folder_name, greyscale_plot=False):
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
            for horizontal in range(0, num_trans_hor_pixel):
                gray_sum = sum(gray_image[start_vertical:end_vertical, horizontal])
                grey_values.append(gray_sum)
                horizontal_pos.append(horizontal)

            # grey_values now contains the sums of grayscale values for each horizontal position in the specified vertical range.
            # print(grey_values)

            # Sliding window parameters
            window_width = 100
            window_interval = 1

            # Calculate the sum of values for each window with a 100-unit interval
            # window_sums = [sum(grey_values[i:i + window_width]) for i in
            #                range(0, len(grey_values) - window_width + 1, window_interval)]
            #
            # # Find the index of the window with the largest sum
            # largest_sum_index = max(range(len(window_sums)), key=window_sums.__getitem__)

            # Print the horizontal position of the largest sum window and the second-largest sum window
            # print('Horizontal position of the largest sum window:', horizontal_pos[largest_sum_index * window_interval])

            # Calculate the sum of values for three sliding windows at the same time
            window_sums = [
                sum(grey_values[i:i + window_width-1]) +
                sum(grey_values[i + window_width :i + 2 * window_width -1]) +
                sum(grey_values[i + 2 * window_width :i + 3 * window_width -1])
                for i in range(0, len(grey_values) - 3 * window_width -1, window_interval)
            ]

            # Find the index of the window with the largest sum
            largest_sum_index = max(range(len(window_sums)), key=window_sums.__getitem__)

            # Calculate the horizontal positions of the left and right windows adjacent to the largest window
            left_window_pos = largest_sum_index * window_interval - window_width
            right_window_pos = left_window_pos + 2 * window_width

            # Check if the largest sum window is at the first window width position
            if left_window_pos == 0:
                left_window_pos = None

            # Check if the largest sum window is at the last window width position
            if right_window_pos >= len(horizontal_pos):
                right_window_pos = None

            # Print the horizontal position of the largest sum window and the positions of adjacent left and right windows
            print('Horizontal position of the largest sum window:', horizontal_pos[largest_sum_index * window_interval])
            print('Horizontal position of the left window:', left_window_pos)
            print('Horizontal position of the right window:', right_window_pos)

            if greyscale_plot:
                # Plot the sliding window with the largest sum-up value
                plt.plot(horizontal_pos, grey_values, label='Gray Values')
                plt.axvspan(
                    horizontal_pos[largest_sum_index * window_interval],
                    horizontal_pos[largest_sum_index * window_interval] + window_width,
                    color='red', alpha=0.3, label='Max Sliding Window'
                )


                # Add labels and title
                plt.xlabel('Horizontal Position')
                plt.ylabel('Gray Value Sum')
                plt.title('Sliding Window with the Maximum Value')
                plt.legend()

                # Show the plot
                plt.show()

            # Exclude the largest sum window from grey_values
            excluded_window_start = max(0, largest_sum_index * window_interval)
            excluded_window_end = min(len(grey_values), excluded_window_start + window_width)
            # Exclude the values between excluded_window_start and excluded_window_end from grey_values
            for i in range(excluded_window_start, excluded_window_end):
                if i < len(grey_values):
                    grey_values[i] = 0

            # Calculate the sum of values for each window with a 100-unit interval in the modified grey_values
            new_window_sums = [sum(grey_values[i:i + window_width]) for i in
                               range(0, len(grey_values) - window_width + 1, window_interval)]

            # Find the index of the window with the largest sum in the modified grey_values
            new_largest_value_index = max(range(len(new_window_sums)), key=new_window_sums.__getitem__)
            # Print the horizontal position of the largest sum window and the second-largest sum window
            print('Horizontal position of the second largest sum window:', horizontal_pos[new_largest_value_index * window_interval])

            print("----------")


            if greyscale_plot:
                # Plot the sliding window with the largest value in the modified grey_values
                plt.plot(horizontal_pos, grey_values, label='Gray Values')
                plt.axvspan(
                    horizontal_pos[new_largest_value_index * window_interval],
                    horizontal_pos[new_largest_value_index * window_interval] + window_width,
                    color='blue', alpha=0.3, label='New Max Sliding Window'
                )

                # Add labels and title
                plt.xlabel('Horizontal Position')
                plt.ylabel('Gray Value Sum')
                plt.title('Sliding Window with the Second Largest Value')
                plt.legend()
                # Show the plot
                plt.show()

            if abs(largest_sum_index* window_interval - new_largest_value_index * window_interval) <= 1.5*window_width:
                #the largest and the second largest value is next to each other
                veh_to_lane_center = 500
            else:


                cam_to_lane_center = largest_sum_index * dpp_hor / 10
                veh_to_lane_center = cam_to_lane_center + veh_width / 20 + 10
                print("Veh to lane center:", veh_to_lane_center, "cm")

           

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
