
import os, os.path
import time
import json
import sys
import pandas as pd
import datetime
from helpers import event_timestamp, copy_files_to_subfolders, file_recategory, extract_images_from_bag, image_processed_folder_name, extract_gps_to_csv, get_event_start_stop_time, get_video_file_name,extract_images_to_video, sync_primary_and_secondary_images,filtered_event_timestamp, extract_can_for_sub_category, get_lane_deviation
from lane_detection import distance2lane
from p_transform import p_transform
import cv2
import logging

log = logging.getLogger(__name__)
log_file_path = os.path.join(os.getcwd(), "post_process_logs")
if not os.path.exists(log_file_path):
    os.makedirs(log_file_path, exist_ok=True)
log_filename = log_file_path + '/post_process_' + datetime.datetime.now().strftime('%d-%m-%Y_%H_%M_%S.log')
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s', filename=log_filename, filemode='w')


#DATA CLASSIFICATION
def perform_data_classification(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path):
    joystick_file_path = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, "event_signal/joystick.txt")
    timestamp_lists_all_subcategories = event_timestamp(joystick_file_path)
    source_folder = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type)
    log.info('Data recategory started')
    process_start_time = time.time()
    copy_files_to_subfolders(source_folder, sub_categories_to_classify)
    for sub_category in sub_categories_to_classify:
        timestamp_list = timestamp_lists_all_subcategories[sub_category]
        file_recategory(timestamp_list, source_folder, sub_category)
    log.info('Data recategory finished')
    end_time = time.time()
    execution_time = int((end_time - process_start_time)/60)
    log.info('Execution time: {} mins'.format(execution_time))

#IMAGE EXTRACTION
def perform_image_extraction_for_camera(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path, target_folder_parent_path):
    source_folder = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type)
    log.info('Image extraction started for: {}'.format(data_classification_folder_type))
    process_start_time = time.time()
    for sub_category in sub_categories_to_classify:
        target_folder_path = os.path.join(target_folder_parent_path, subject_id, alcohol_session_name, sub_category)
        sub_category_folder = os.path.join(source_folder, sub_category)
        sub_category_runs = len([folder for folder in os.listdir(sub_category_folder) if os.path.isdir(os.path.join(sub_category_folder, folder))] )
        for sub_category_run_number in range(1, sub_category_runs+1):
            save_folder_for_camera_images = os.path.join(target_folder_path, sub_category + '_' + str(sub_category_run_number))
            camera_input_folder = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number))
            camera_output_folder = os.path.join(save_folder_for_camera_images, "images", image_processed_folder_name(data_classification_folder_type))
            is_camera_flipped_vert = True if data_classification_folder_type in ["images1","images2", "images3", "images4"] else False
            is_camera_flipped_hor = True if data_classification_folder_type in ["images1", "images2", "images3", "images4"] else False
            start_time, stop_time = get_event_start_stop_time(camera_input_folder)
            if os.path.exists(camera_output_folder):
                for file in os.listdir(camera_output_folder):
                    file_path = os.path.join(camera_output_folder, file)
                    os.remove(file_path)
            extract_images_from_bag(camera_input_folder, camera_output_folder, is_camera_flipped_vert, is_camera_flipped_hor, start_time, stop_time)
    log.info('Image extraction finished for: {}'.format(data_classification_folder_type))
    end_time = time.time()
    execution_time = int((end_time - process_start_time)/60)
    log.info('Execution time: {} mins'.format(execution_time))

#IMAGE SYNC

def perform_image_sync(subject_id, alcohol_session_name, target_folder_path):
    source_folder = os.path.join(target_folder_path, subject_id, alcohol_session_name)
    sub_categories_to_classify = os.listdir(source_folder)
    log.info('Image Sync started for all cameras')
    process_start_time = time.time()
    for sub_category in sub_categories_to_classify:
        sub_category_folder = os.path.join(source_folder, sub_category)
        sub_category_runs = len([folder for folder in os.listdir(sub_category_folder) if os.path.isdir(os.path.join(sub_category_folder, folder))] )
        for sub_category_run_number in range(1, sub_category_runs +1):
            #All the images folders in the subcategory without synced in the end of the name
            images_dirs = [folder for folder in os.listdir(os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number), "images")) if not folder.endswith(".csv") and not folder.endswith("transformed")]
            if len(images_dirs) == 1:
                continue
            else:
                if "image_left" in images_dirs:
                    primary_sync_ref_image = "image_left"
                else:
                    primary_sync_ref_image = images_dirs[0]
                primary_sync_ref_image_path = os.path.join(source_folder, sub_category, sub_category + '_' + str(sub_category_run_number), "images",primary_sync_ref_image)
                for img_folder in images_dirs:
                    if img_folder != primary_sync_ref_image:
                        secondary_sync_ref_image_path = os.path.join(source_folder, sub_category, sub_category + '_' + str(sub_category_run_number), "images",img_folder)
                        parent_folder = os.path.join(source_folder, sub_category, sub_category + '_' + str(sub_category_run_number), "images")
                        sync_primary_and_secondary_images(primary_sync_ref_image, primary_sync_ref_image_path, img_folder, secondary_sync_ref_image_path, parent_folder)
    log.info('Image Sync completed for all cameras')
    end_time = time.time()
    execution_time = int((end_time - process_start_time)/60)
    log.info('Execution time: {} mins'.format(execution_time))
#CAN EXTRACTION
def perform_can_extraction(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path, target_folder_parent_path):
    source_folder = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type)
    joystick_file_path = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, "event_signal/joystick.txt")
    timestamp_lists_all_subcategories = event_timestamp(joystick_file_path)
    can_file_name = os.listdir(source_folder)[0]
    can_input_file_path = os.path.join(source_folder, can_file_name)
    log.info('CAN extraction started for: {}'.format(data_classification_folder_type))
    process_start_time = time.time()
    for sub_category in sub_categories_to_classify:
        timestamp_list = timestamp_lists_all_subcategories[sub_category]
        filtered_timestamps = filtered_event_timestamp(timestamp_list)
        paired_filtered_list = [filtered_timestamps[i:i+2] for i in range(0, len(filtered_timestamps), 2)]
        target_folder_path = os.path.join(target_folder_parent_path, subject_id, alcohol_session_name, sub_category, sub_category)
        extract_can_for_sub_category(can_input_file_path, target_folder_path, paired_filtered_list)
    log.info('CAN extraction finished for: {}'.format(data_classification_folder_type))
    end_time = time.time()
    execution_time = int((end_time - process_start_time)/60)
    log.info('Execution time: {} mins'.format(execution_time))

#GPS EXTRACTION
def perform_gps_extraction(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path, target_folder_parent_path):
    source_folder = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type)
    log.info('GPS extraction started for: {}'.format(data_classification_folder_type))
    process_start_time = time.time()
    for sub_category in sub_categories_to_classify:
        target_folder_path = os.path.join(target_folder_parent_path, subject_id, alcohol_session_name, sub_category)
        sub_category_folder = os.path.join(source_folder, sub_category)
        sub_category_runs = len([folder for folder in os.listdir(sub_category_folder) if os.path.isdir(os.path.join(sub_category_folder, folder))] )

        for sub_category_run_number in range(1, sub_category_runs+1):
            save_folder_for_gps = os.path.join(target_folder_path, sub_category + '_' + str(sub_category_run_number))
            gps_input_folder = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number))
            gps_output_folder = os.path.join(save_folder_for_gps, "gps")
            start_time, stop_time = get_event_start_stop_time(gps_input_folder)
            if os.path.exists(gps_output_folder):
                for file in os.listdir(gps_output_folder):
                    file_path = os.path.join(gps_output_folder, file)
                    os.remove(file_path)
            extract_gps_to_csv(gps_input_folder, gps_output_folder, start_time, stop_time)
    log.info('GPS extraction finished for: {}'.format(data_classification_folder_type))
    end_time = time.time()
    execution_time = int((end_time - process_start_time)/60)
    log.info('Execution time: {} mins'.format(execution_time))

#VIDEO EXTRACTION USING FRAMES
def perform_video_extraction_using_frames(subject_id, alcohol_session_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path, target_folder_parent_path):
    source_folder = os.path.join(target_folder_parent_path, subject_id, alcohol_session_name)
    image_folder_name = image_processed_folder_name(data_classification_folder_type)
    log.info('Video extraction started for: {}'.format(data_classification_folder_type))
    process_start_time = time.time()
    for sub_category in sub_categories_to_classify:
        sub_category_folder = os.path.join(source_folder, sub_category)
        sub_category_runs = len([folder for folder in os.listdir(sub_category_folder) if os.path.isdir(os.path.join(sub_category_folder, folder))] )
        for sub_category_run_number in range(1, sub_category_runs + 1):
            video_input_folder = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number),"images", image_folder_name)
            video_output_folder = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number), "videos")
            if os.path.exists(video_output_folder):
                existing_video_filename = get_video_file_name(data_classification_folder_type)
                for file in os.listdir(video_output_folder):
                    if file == existing_video_filename:
                        file_path = os.path.join(video_output_folder, file)
                        os.remove(file_path)
            extract_images_to_video(video_input_folder, video_output_folder, data_classification_folder_type)
    log.info('Video extraction finished for: {}'.format(data_classification_folder_type))
    end_time = time.time()
    execution_time = int((end_time - process_start_time)/60)
    log.info('Execution time: {} mins'.format(execution_time))
#PERFORM PERSPECTIVE TRANSFORMATION
def perform_lane_deviation_calculation(subject_id, alcohol_session_name, data_classification_folder_type, sub_categories_to_classify, target_folder_parent_path, lane_deviation_parameters,mat_hor_len):
    log.info('Lane deviation calculation started for: {}'.format(data_classification_folder_type))
    process_start_time = time.time()
    image_folder_name = image_processed_folder_name(data_classification_folder_type)
    skip_perpective_transformation = lane_deviation_parameters['skip_perpective_transformation']
    top_left = lane_deviation_parameters['corner_points']['top_left']
    top_right = lane_deviation_parameters['corner_points']['top_right']
    bottom_left = lane_deviation_parameters['corner_points']['bottom_left']
    bottom_right = lane_deviation_parameters['corner_points']['bottom_right']
    corner_positions = [top_left, top_right, bottom_left, bottom_right]
    calibration_img_path = lane_deviation_parameters['calibration_img_path']
    calibration_img = cv2.imread(calibration_img_path)
    source_folder = os.path.join(target_folder_parent_path, subject_id, alcohol_session_name)
    flipped = False
    if image_folder_name == "image_left":
        flipped = True
    for sub_category in sub_categories_to_classify:
        sub_category_folder = os.path.join(source_folder, sub_category)
        sub_category_runs = len([folder for folder in os.listdir(sub_category_folder) if os.path.isdir(os.path.join(sub_category_folder, folder))] )
        for sub_category_run_number in range(1, sub_category_runs + 1):
            camera_input_folder = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number),"images", image_folder_name)
            camera_output_folder = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number),"images",image_folder_name + "_transformed")
            lane_deviation_output = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number),"lane_deviation")

            p_transform(corner_positions, calibration_img, camera_input_folder, camera_output_folder, flipped)
            distance2lane(camera_output_folder,lane_deviation_output, image_folder_name, mat_hor_len)
            if os.path.exists(camera_output_folder):
                # delete the transformed images folder
                for file in os.listdir(camera_output_folder):
                    file_path = os.path.join(camera_output_folder, file)
                    os.remove(file_path)
                os.rmdir(camera_output_folder)
    log.info('Lane deviation calculation finished for: {}'.format(data_classification_folder_type))
    end_time = time.time()
    execution_time = int((end_time - process_start_time)/60)
    log.info('Execution time: {} mins'.format(execution_time))

def combine_lane_deviation_output_from_side_cams(subject_id, alcohol_session_name, sub_categories_to_classify, target_folder_parent_path):
    log.info('Lane Deviation combination process started')
    process_start_time = time.time()
    source_folder = os.path.join(target_folder_parent_path, subject_id, alcohol_session_name)
    for sub_category in sub_categories_to_classify:
        sub_category_folder = os.path.join(source_folder, sub_category)
        sub_category_runs = len([folder for folder in os.listdir(sub_category_folder) if os.path.isdir(os.path.join(sub_category_folder, folder))] )
        for sub_category_run_number in range(1, sub_category_runs + 1):
            lane_deviation_output_left = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number),"lane_deviation", "image_left_distance_to_lane.csv")
            lane_deviation_output_right = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number),"lane_deviation", "image_right_distance_to_lane.csv")

            if os.path.exists(lane_deviation_output_left) and os.path.exists(lane_deviation_output_right):
                lane_deviation_output_combined = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number),"lane_deviation", "Lane_Deviation.csv")
                df_left = pd.read_csv(lane_deviation_output_left)
                #Rename the columns to avoid confusion
                df_left = df_left.rename(columns={"Distance_to_Lane (cm)": "Distance_to_Lane_left (cm)","Timestamp": "Timestamp_left","Date time": "Date time_left"})
                df_right = pd.read_csv(lane_deviation_output_right)
                df_right = df_right.rename(columns={"Distance_to_Lane (cm)": "Distance_to_Lane_right (cm)","Timestamp": "Timestamp_right","Date time": "Date time_right"})
                df_combined = pd.concat([df_left, df_right], axis=1)
                #Add a column for the which contains the lane deviation and it is calculated by passing the distance to lane left and right to the function get_lane_deviation() and store the returned value in the new column
                df_combined['Lane_Deviation (cm)'] = df_combined.apply(lambda x: get_lane_deviation(x['Distance_to_Lane_left (cm)'], x['Distance_to_Lane_right (cm)']), axis=1)
                #Add a column for the difference in time between the two timestamps
                df_combined['Time_Difference (ms)'] = (abs(df_combined['Timestamp_left'] - df_combined['Timestamp_right']))/1000000
                df_combined['Left_Lane_Boundary'] = 180
                df_combined['Right_Lane_Boundary'] = -180
                df_combined.to_csv(lane_deviation_output_combined, index=False)
                log.info("Lane deviation output combined for {} - {}".format(sub_category, sub_category_run_number))
                #Print the average lane deviation
                log.info("Average lane deviation:{}".format(df_combined['Lane_Deviation (cm)'].mean()))
                #Print the average time difference
                log.info("Average time difference: {}".format(df_combined['Time_Difference (ms)'].mean()))
    log.info('Lane Deviation combination process finished')
    end_time = time.time()
    execution_time = int((end_time - process_start_time)/60)
    log.info('Execution time: {} mins'.format(execution_time))


if __name__ == '__main__':
    # Read the arguments from the json file who's path is given as the first argument


    start_time = time.time()
    with open(sys.argv[1]) as json_file:
        config = json.load(json_file)
        base_config = config['base_config']
        subject_id = base_config['subject_id']
        alcohol_session_name = base_config['alcohol_session_name']
        timestamped_folder_name = base_config['timestamped_folder_name']
        source_folder_path = base_config['source_folder_path']
        target_folder_path = base_config['target_folder_path']
        hor_length_of_calibration_mat = base_config.get('length_of_calibration_mat', 1455)
        sync_required = base_config.get('sync_required',False)

        log.info('Starting post processing for subject: {} and session: {}'.format(subject_id, alcohol_session_name))

        dict_of_runs = config['execution_specific_config']
        # log_file_path = os.path.join(source_folder_path, subject_id, alcohol_session_name)

        for data_classification_folder_type, run in dict_of_runs.items():
            if "execute_data_classification" in run and run['execute_data_classification'] == True:
                perform_data_classification(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, run['sub_categories_to_classify'], source_folder_path)

        for data_classification_folder_type, run in dict_of_runs.items():
            if "execute_can_extraction" in run and run["execute_can_extraction"] == True:
                perform_can_extraction(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, run['sub_categories_to_classify'], source_folder_path, target_folder_path)

        for data_classification_folder_type, run in dict_of_runs.items():
            if "execute_gps_extraction" in run and run["execute_gps_extraction"] == True:
                perform_gps_extraction(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, run['sub_categories_to_classify'], source_folder_path, target_folder_path)

        for data_classification_folder_type, run in dict_of_runs.items():
            if "execute_image_extraction" in run and  run["execute_image_extraction"] == True:
                perform_image_extraction_for_camera(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, run['sub_categories_to_classify'], source_folder_path, target_folder_path)

        if sync_required:
            log.info("Syncing images")
            perform_image_sync(subject_id, alcohol_session_name, target_folder_path)

        for data_classification_folder_type, run in dict_of_runs.items():
            if "execute_video_extraction" in run and run["execute_video_extraction"] == True:
                perform_video_extraction_using_frames(subject_id, alcohol_session_name, data_classification_folder_type, run['sub_categories_to_classify'], source_folder_path, target_folder_path)

        for data_classification_folder_type, run in dict_of_runs.items():
            if "execute_lane_deviation_calculation" in run and run["execute_lane_deviation_calculation"] == True:
                lane_deviation_parameters = run['lane_deviation_parameters']
                sub_categories_for_lane_deviation = run['lane_deviation_parameters']['sub_categories_for_lane_deviation']
                perform_lane_deviation_calculation(subject_id, alcohol_session_name, data_classification_folder_type, sub_categories_for_lane_deviation, target_folder_path, lane_deviation_parameters,hor_length_of_calibration_mat)

        for data_classification_folder_type, run in dict_of_runs.items():
            if "execute_lane_deviation_calculation" in run and run["execute_lane_deviation_calculation"] == True and data_classification_folder_type == "images4":
                combine_lane_deviation_output_from_side_cams(subject_id, alcohol_session_name, run['sub_categories_to_classify'], target_folder_path)

        end_time = time.time()
        execution_time = int((end_time - start_time)/60)
        log.info('Complete Data Post Processing Execution time: {} mins'.format(execution_time))