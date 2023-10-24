#!/usr/bin/env python3

import rosbag
import time
import os
import cv2
from cv_bridge import CvBridge
VIDEO_FRAMERATE = 30
MJPEG_VIDEO = 1
RAWIMAGE_VIDEO = 2
VIDEO_CONVERTER_TO_USE = "ffmpeg" # or you may want to use "avconv"
# RECORDING_FOLDER_NAME = '22-09-23_15:25:12'
# SOURCE_CAMERA_BAG_FOLDER = "/home/iac_user/DATA_COLLECTION/" + RECORDING_FOLDER_NAME
# SAVE_FOLDER_FOR_CAMERA_IMAGES = '/home/iac_user/PROCESSED_DATA_COLLECTION/' + RECORDING_FOLDER_NAME

# SUB_FOLDER_NAME = 'Driving backward/'
# # SUB_FOLDER_NAME = 'Driving forward/'
# # SUB_FOLDER_NAME = 'Eye Tracking/'
# # SUB_FOLDER_NAME = 'Parking/'

# SUB_SUB_FOLDER_NAME = 'Driving backward_'
# # SUB_SUB_FOLDER_NAME = 'Driving forward_'
# # SUB_SUB_FOLDER_NAME = 'Eye Tracking_'
# # SUB_SUB_FOLDER_NAME = 'Parking_'

# sub_sub_folder_ind_list = [4,5]

# for i in range(len(sub_sub_folder_ind_list)):

#     SUB_SUB_FOLDER_IND = str(sub_sub_folder_ind_list[i])

#     # SUB_SUB_FOLDER_IND = '3' # TODO: change the subfolder ind

#     image_ind_list = [1,2,3,4]

#     for j in range(len(image_ind_list)):
#         IMAGE_FOLDER = 'images' + str(image_ind_list[j]) + '/'

#         SOURCE_CAMERA_BAG_FOLDER = "/home/iac_user/post-process/Post-SubjectLL/Baseline/only_driving/17-10-23_10-50-24/" + IMAGE_FOLDER + SUB_FOLDER_NAME + SUB_SUB_FOLDER_NAME +SUB_SUB_FOLDER_IND
# # /home/iac_user/post-process/test/bag-eye-track
# # /media/iac_user/ImDrive_Org/TestLL/Baseline/new_eye_tracking/17-10-23_11-20-48

#         SAVE_FOLDER_FOR_CAMERA_IMAGES = '/home/iac_user/post-process/1017test/Driving/' + SUB_FOLDER_NAME + SUB_SUB_FOLDER_NAME + SUB_SUB_FOLDER_IND

def combine_bags_to_video(input_bag_folder, output_video_path, camera_topic):
    # Initialize video writer
    codec = cv2.VideoWriter_fourcc('M','J','P','G')  # Use appropriate codec
    fps = VIDEO_FRAMERATE  # Frames per second
    video_writer = None
    bridge = CvBridge()
    counter = 0
    for bag_file in sorted(os.listdir(input_bag_folder)):
        counter +=1 
        if counter == 15:
            break
        if bag_file.endswith(".bag"):
            bag_path = os.path.join(input_bag_folder, bag_file)
            bag = rosbag.Bag(bag_path)
            print("Processing:", bag_path)

            for _, msg, _ in bag.read_messages(topics=[camera_topic]):
                frame = bridge.compressed_imgmsg_to_cv2(msg)
                frame = cv2.flip(frame, -1)
                if video_writer is None:
                    height, width, _ = frame.shape
                    video_writer = cv2.VideoWriter(output_video_path, codec, fps, (width, height))

                video_writer.write(frame)

            bag.close()

    if video_writer is not None:
        video_writer.release()
        print("Video conversion complete. Output file:", output_video_path)
    else:
        print("No bag files found for conversion.")

if __name__ == '__main__': 
    start_time = time.time()

    sub_folder_name = 'Driving backward'
    # sub_folder_name = 'Driving forward'
    # sub_folder_name = 'Eye Tracking'
    # sub_folder_name = 'Parking'

    SUB_FOLDER_NAME = sub_folder_name + '/'

    SUB_SUB_FOLDER_NAME = sub_folder_name +'_'

    #TODO: change the subfolder ind into a len(sub_sub_folder)
    sub_sub_folder_ind_list = [1]

    for i in range(len(sub_sub_folder_ind_list)):

        SUB_SUB_FOLDER_IND = str(sub_sub_folder_ind_list[i])

        # SUB_SUB_FOLDER_IND = '3' # TODO: change the subfolder ind

        image_ind_list = [1,2,3,4]

        for j in range(len(image_ind_list)):
            IMAGE_FOLDER = 'images' + str(image_ind_list[j]) + '/'

            SOURCE_CAMERA_BAG_FOLDER = "/home/iac_user/post-process/Post-SubjectLL/Baseline/only_driving/17-10-23_10-50-24/" + IMAGE_FOLDER + SUB_FOLDER_NAME + SUB_SUB_FOLDER_NAME +SUB_SUB_FOLDER_IND
    # /home/iac_user/post-process/test/bag-eye-track
    # /media/iac_user/ImDrive_Org/TestLL/Baseline/new_eye_tracking/17-10-23_11-20-48

            SAVE_FOLDER_FOR_CAMERA_IMAGES = '/home/iac_user/post-process/1017test/Driving/' + SUB_FOLDER_NAME + SUB_SUB_FOLDER_NAME + SUB_SUB_FOLDER_IND

            camera1_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/videos/video1"
            camera2_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/videos/video2"
            camera1_topic = '/camera1/usb_cam1/image_raw/compressed'
            camera2_topic = '/camera2/usb_cam2/image_raw/compressed'
            # camera1_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images1"
            # camera2_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images2"
            camera1_input_folder = SOURCE_CAMERA_BAG_FOLDER 
            camera2_input_folder = SOURCE_CAMERA_BAG_FOLDER 

            camera3_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/videos/video3"
            camera4_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/videos/video4"
            camera3_topic = '/camera3/usb_cam3/image_raw/compressed'
            camera4_topic = '/camera4/usb_cam4/image_raw/compressed'
            # camera3_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images3"
            # camera4_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images4"
            camera3_input_folder = SOURCE_CAMERA_BAG_FOLDER
            camera4_input_folder = SOURCE_CAMERA_BAG_FOLDER

            opt_files=[f for f in os.listdir(camera1_input_folder) if f[-4:] == ".bag"]
            opt_files=sorted(opt_files)	
            os.makedirs(SAVE_FOLDER_FOR_CAMERA_IMAGES+"/videos",exist_ok=True)

            combine_bags_to_video(camera1_input_folder,camera1_output_folder+".mp4",camera1_topic)

            combine_bags_to_video(camera2_input_folder,camera2_output_folder+".mp4",camera2_topic)

            combine_bags_to_video(camera3_input_folder,camera3_output_folder+".mp4",camera3_topic)

            combine_bags_to_video(camera4_input_folder,camera4_output_folder+".mp4",camera4_topic)  

            print(SAVE_FOLDER_FOR_CAMERA_IMAGES, 'video generated')
            print('---------------------------------------------')
            print('---------------------------------------------')

        print("finished")

    end_time = time.time()

    print("Time elapsed: ", end_time - start_time, "seconds")
            
            