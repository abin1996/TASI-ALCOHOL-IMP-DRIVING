
import os, os.path
import time
import json
import sys
import rosbag
import pandas as pd
import datetime
from helpers import event_timestamp, copy_files_to_subfolders, file_recategory, extract_images_from_bag, image_processed_folder_name, extract_gps_to_csv, get_event_start_stop_time, extract_bags_to_video



def perform_data_classification(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path): 

    joystick_file_path = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, "event_signal/joystick.txt")
    timestamp_lists_all_subcategories = event_timestamp(joystick_file_path)
    source_folder = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type)
    copy_files_to_subfolders(source_folder, sub_categories_to_classify)
    
    for sub_category in sub_categories_to_classify:
        timestamp_list = timestamp_lists_all_subcategories[sub_category]
        file_recategory(timestamp_list, source_folder, sub_category)
    print('Data recategory finished')
   
def perform_image_extraction_for_camera(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path, target_folder_parent_path):

    source_folder = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type)
    for sub_category in sub_categories_to_classify:
        target_folder_path = os.path.join(target_folder_parent_path, subject_id, alcohol_session_name, sub_category)
        
        sub_category_folder = os.path.join(source_folder, sub_category)
        sub_category_runs = len([folder for folder in os.listdir(sub_category_folder) if os.path.isdir(os.path.join(sub_category_folder, folder))] )
        
        for sub_category_run_number in range(1, sub_category_runs+1):
            save_folder_for_camera_images = os.path.join(target_folder_path, sub_category + '_' + str(sub_category_run_number))
            camera_input_folder = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number))
            camera_output_folder = os.path.join(save_folder_for_camera_images, image_processed_folder_name(data_classification_folder_type))
            is_camera_flipped = True if data_classification_folder_type == "images1" else False
            session_frames_count = 0
            frame_gaps_list = []
            prev_frame = None
            prev_frame_time = 0
            start_time, stop_time = get_event_start_stop_time(camera_input_folder)
            if os.path.exists(camera_output_folder):
                for file in os.listdir(camera_output_folder):
                    file_path = os.path.join(camera_output_folder, file)
                    os.remove(file_path) 
            for (root, dirs, files) in os.walk(camera_input_folder):
                for file in sorted(files):
                    if file.endswith(".bag"):
                        bag_num = (file.split('_')[2]).split('.')[0]
                        bag = rosbag.Bag(os.path.join(root, file))
                        # print("Number of frames in bag number " + bag_num + " : " + str(missing_frames_in_bag))
                        frame_gaps, session_frames_count, prev_frame, prev_frame_time, total_missing_frames = extract_images_from_bag(bag, camera_output_folder, is_camera_flipped, bag_num, start_time, stop_time, session_frames_count, prev_frame, prev_frame_time)
                        print("Duration bag number " + bag_num + " : " + str(bag.get_end_time() - bag.get_start_time()) + ", Missing frames: " + str(total_missing_frames))
                        frame_gaps_list.extend(frame_gaps)
                        bag.close()
            
            #Store the number of missing frames every bag in a csv using pandas
            df = pd.DataFrame(frame_gaps_list, columns =['Bag Number', 'Start of missing frame','Number of Missing Frames', 'Time interval'])
            df.to_csv(os.path.join(save_folder_for_camera_images, image_processed_folder_name(data_classification_folder_type) + "_missing_frames.csv"), index=False)

def perform_gps_extraction(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path, target_folder_parent_path):
    source_folder = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type)
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

def perform_video_extraction(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path, target_folder_parent_path):
    source_folder = os.path.join(source_folder_path, subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type)
    for sub_category in sub_categories_to_classify:
        target_folder_path = os.path.join(target_folder_parent_path, subject_id, alcohol_session_name, sub_category)
        sub_category_folder = os.path.join(source_folder, sub_category)
        sub_category_runs = len([folder for folder in os.listdir(sub_category_folder) if os.path.isdir(os.path.join(sub_category_folder, folder))] )
        
        for sub_category_run_number in range(1, sub_category_runs+1):
            save_folder_for_video = os.path.join(target_folder_path, sub_category + '_' + str(sub_category_run_number))
            video_input_folder = os.path.join(sub_category_folder, sub_category + '_' + str(sub_category_run_number))
            video_output_folder = os.path.join(save_folder_for_video, "videos")
            start_time, stop_time = get_event_start_stop_time(video_input_folder)
            if os.path.exists(video_output_folder):
                for file in os.listdir(video_output_folder):
                    file_path = os.path.join(video_output_folder, file)
                    os.remove(file_path) 
            extract_bags_to_video(video_input_folder, video_output_folder, data_classification_folder_type, start_time, stop_time)

if __name__ == '__main__':
    # Read the arguments from the json file who's path is given as the first argument
    start_time = time.time()
    with open(sys.argv[1]) as json_file:
        list_of_runs = json.load(json_file)
        for run in list_of_runs:
            subject_id = run['subject_id']
            alcohol_session_name = run['alcohol_session_name']
            timestamped_folder_name = run['timestamped_folder_name']
            data_classification_folder_type = run['data_classification_folder_type']
            sub_categories_to_classify = run['sub_categories_to_classify']
            source_folder_path = run['source_folder_path']

            if run['execute_data_classification'] == True:
                perform_data_classification(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path)

            if "execute_image_extraction" in run and  run["execute_image_extraction"] == True and "target_folder_path" in run:
                target_folder_parent_path = run['target_folder_path']
                perform_image_extraction_for_camera(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path, target_folder_parent_path)

            if "execute_gps_extraction" in run and run["execute_gps_extraction"] == True:
                target_folder_parent_path = run['target_folder_path']
                perform_gps_extraction(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path, target_folder_parent_path)

            if "execute_video_extraction" in run and run["execute_video_extraction"] == True:
                target_folder_parent_path = run['target_folder_path']
                perform_video_extraction(subject_id, alcohol_session_name, timestamped_folder_name, data_classification_folder_type, sub_categories_to_classify, source_folder_path, target_folder_parent_path)
        end_time = time.time()
        execution_time = end_time - start_time
        print('Execution time',execution_time)