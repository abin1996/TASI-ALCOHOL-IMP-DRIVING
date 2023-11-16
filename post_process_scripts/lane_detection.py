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
                       image = cv2.imread(full_path)

            # Extract the timestamp from the filename
            timestamp = extract_timestamp(filename)
            # Convert the timestamp to a datetime object
            datetime_obj = timestamp_to_datetime(timestamp)

            print('Imgage:',full_path)
            # print('Timestamp:', timestamp)

            # 1. Convert the RGB image to grayscale
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # define a kernel size and apply Gaussian smoothing
            kernel_size = 3
            blur_gray = gaussian_blur(gray_image, kernel_size)

            # output_path = full_path + '_greyscale'
            # Save the transformed image
            # cv2.imwrite(output_path, gray_image)

            # define our parameters for Canny and apply
            # Otu's threshold selection method
            high_threshold, thresh_im = cv2.threshold(blur_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            low_threshold = 0.5 * high_threshold
            # low_threshold = 50
            # high_threshold = 150
            edges = canny(blur_gray, low_threshold, high_threshold)

            # print('EDGES:',edges)
            if greyscale_plot:
                plt.figure()
                plt.imshow(thresh_im)
                plt.show()

            # 2. Define the vertical position range
            start_vertical = 450
            end_vertical = 550

            # 3. Create an empty list to store the gray values
            grey_values = []
            horizontal_pos = []

            gray_image = thresh_im

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
            window_sums = [sum(grey_values[i:i + window_width]) for i in
                           range(0, len(grey_values) - window_width + 1, window_interval)]

            # Find the index of the window with the largest sum
            largest_sum_index = max(range(len(window_sums)), key=window_sums.__getitem__)

            # Calculate the horizontal positions of the left and right windows adjacent to the largest window
            left_window_pos = largest_sum_index * window_interval - window_width
            right_window_pos = left_window_pos + 2* window_width

            # Calculate the sum of the window on the left side of the maximum sliding window
            max_left = sum(grey_values[left_window_pos - window_width:left_window_pos])

            # Calculate the sum of the window on the right side of the maximum sliding window
            max_right = sum(grey_values[right_window_pos + 1:right_window_pos + window_width + 1])

            # Save the maximum sum value as max_window
            max_window = window_sums[largest_sum_index]

            # Check if the max window is on the left boundary of the horizontal axis
            if left_window_pos <= 0:
                max_left = None

            # Check if the max window is on the right boundary of the horizontal axis
            if right_window_pos >= len(horizontal_pos) - 1:
                max_right = None

            if max_left != None:
                diff_left = abs(max_window - max_left)

            if max_right != None:
                diff_right = abs(max_window - max_right)

            # Print the horizontal position of the largest sum window and the positions of adjacent left and right windows
            print('Horizontal position of the left window:', left_window_pos)
            print('Horizontal position of the largest sum window:', horizontal_pos[largest_sum_index * window_interval])
            print('Horizontal position of the right window:', right_window_pos)

            print(f'Sum of the window on the left side: {max_left}, Left differece: {diff_left}')
            print('Maximum sum value:', max_window)
            print(f'Sum of the window on the right side: {max_right}, Right differece: {diff_right}')
            print('------------------------------')



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

            #TODO: CHANGE THE CONDITION TO EXCLUDE THE BOUNDARY CASE
            # max_grey_value_ind = largest_sum_index * window_interval + window_width//2
            # max_grey_value_ind = largest_sum_index * window_interval

            filter_thr = 0.5

            if diff_left/max_window<= filter_thr or diff_right/max_window <= filter_thr:
                #indicates no huge difference between the max window and the neighbor windows
                veh_to_lane_center = 500
                print("Veh to lane center:", veh_to_lane_center, "cm (NO LANE DETECTED!)")

            elif max_left == None and diff_right/max_window >= filter_thr:
                cam_to_lane_center = largest_sum_index * dpp_hor / 10
                veh_to_lane_center = cam_to_lane_center + veh_width / 20 + 10
                print("Veh to lane center:", veh_to_lane_center, "cm")

            elif max_right == None and diff_left/max_window >= filter_thr:
                cam_to_lane_center = largest_sum_index * dpp_hor / 10
                veh_to_lane_center = cam_to_lane_center + veh_width / 20 + 10
                print("Veh to lane center:", veh_to_lane_center, "cm")
            else:
                cam_to_lane_center = largest_sum_index * dpp_hor / 10
                veh_to_lane_center = cam_to_lane_center + veh_width / 20 + 10
                print("Veh to lane center:", veh_to_lane_center, "cm")
            #
            # Create a dictionary to store data for this image
            data_dict = {
                'Date time': datetime_obj,
                'Timestamp': timestamp,
                'Distance_to_Lane (cm)': veh_to_lane_center
            }
            data_list.append(data_dict)


    # Save the data as a CSV file
    csv_file_path = os.path.join(folder_path, 'distance_to_lane_v3.csv')

    with open(csv_file_path, mode='w', newline='') as csv_file:
        fieldnames = ['Date time', 'Timestamp','Distance_to_Lane (cm)']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        for data_dict in data_list:
            writer.writerow(data_dict)

    print('CSV file saved:', csv_file_path)
    
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
