import os
import rosbag
import cv2
from cv_bridge import CvBridge, CvBridgeError

import time

# RECORDING_FOLDER_NAME = '22-09-23_15:25:12'

# SOURCE_CAMERA_BAG_FOLDER = "/home/iac_user/post-process/Post-SubjectLL/Baseline/only_driving" + RECORDING_FOLDER_NAME

# SAVE_FOLDER_FOR_CAMERA_IMAGES = '/home/iac_user/PROCESSED_DATA_COLLECTION/' + RECORDING_FOLDER_NAME

# count = 0

def extract_img(dirs, filename,topic, output_folder, flipped):

    count = 0
    # global count
    bag = rosbag.Bag(os.path.join(dirs, filename))

    for (topic, msg, t) in bag.read_messages():
        bridge = CvBridge()
        try:
            cv_img = bridge.compressed_imgmsg_to_cv2(msg)
            print("here")
        except CvBridgeError as e:
            print(e)
            continue
        if flipped:
            cv_img = cv2.flip(cv2.flip(cv_img, 1), 0)
        filename = os.path.join(output_folder, f'frame_{count}_{t}.png')
        print(filename)
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        cv2.imwrite(filename, cv_img)
        print(count)
        count += 1
    bag.close()


# camera1_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/img1"
# camera2_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/img2"
# camera1_topic = ['/camera1/usb_cam1/image_raw/compressed']
# camera2_topic = ['/camera2/usb_cam2/image_raw/compressed']
# camera1_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images1"
# camera2_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images2"
# # camera_list = [camera1_folder, camera2_folder]

# for (root, dirs, files) in os.walk(camera1_input_folder):
#     for file in sorted(files):
#         print(file)
#         extract_img(root, file, camera1_topic, camera1_output_folder, True)
#         #break
# for (root, dirs, files) in os.walk(camera2_input_folder):
#     for file in sorted(files):
#         print(file)
#         extract_img(root, file, camera2_topic, camera2_output_folder, True)

# camera3_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/img3"
# camera4_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/img4"
# camera3_topic = ['/camera3/usb_cam4/image_raw/compressed']
# camera4_topic = ['/camera4/usb_cam4/image_raw/compressed']
# camera3_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images3"
# camera4_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images4"
# # camera_list = [camera1_folder, camera2_folder]
# for (root, dirs, files) in os.walk(camera3_input_folder):
#     for file in sorted(files):
#         print(file)
#         extract_img(root, file, camera3_topic, camera3_output_folder, True)
#         #break
# for (root, dirs, files) in os.walk(camera4_input_folder):
#     for file in sorted(files):
#         print(file)
#         extract_img(root, file, camera4_topic, camera4_output_folder, True)


if __name__ == '__main__': 

    start_time = time.time()


    # sub_folder_name = 'Driving backward'
    sub_folder_name = 'Driving forward'
    # sub_folder_name = 'Eye Tracking'
    # sub_folder_name = 'Parking'
    

    SUB_FOLDER_NAME = sub_folder_name + '/'

    SUB_SUB_FOLDER_NAME = sub_folder_name +'_'

    SOURCE_FOLDER = '/home/iac_user/post-process/Post-SubjectLL/Baseline/only_driving/17-10-23_10-50-24'

    image_ind_list = [1,2,3,4]

    for i in range(len(image_ind_list)):
        IMAGE_FOLDER = 'images' + str(image_ind_list[i]) + '/'

        SOURCE_CAMERA_BAG_FOLDER = SOURCE_FOLDER +'/'+ IMAGE_FOLDER + sub_folder_name

        num_sub_sub_folder = len([folder for folder in os.listdir(SOURCE_CAMERA_BAG_FOLDER) if os.path.isdir(os.path.join(SOURCE_CAMERA_BAG_FOLDER, folder))])

        for j in range(num_sub_sub_folder):
            SUB_SUB_FOLDER_IND = str(j+1)

            SAVE_FOLDER_FOR_CAMERA_IMAGES = '/home/iac_user/post-process/1017test/Driving/' + SUB_FOLDER_NAME + SUB_SUB_FOLDER_NAME + SUB_SUB_FOLDER_IND

            camera_input_folder = SOURCE_CAMERA_BAG_FOLDER + '/' + SUB_SUB_FOLDER_NAME + SUB_SUB_FOLDER_IND

            camera_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + '/images/' + IMAGE_FOLDER

            camera_topic = ['/camera' + str(image_ind_list[i])+ '/usb_cam'+str(image_ind_list[i]) +'/image_raw/compressed' ]

            for (root, dirs, files) in os.walk(camera_input_folder):
                for file in sorted(files):
                    print(file)
                    extract_img(root, file, camera_topic, camera_output_folder, True)
                    #break  

    end_time = time.time()

    print("Overall time taken: ", end_time - start_time)
