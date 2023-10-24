
if __name__ == '__main__': 
    SUB_FOLDER_NAME = 'Driving backward/'
    # SUB_FOLDER_NAME = 'Driving forward/'
    # SUB_FOLDER_NAME = 'Eye Tracking/'
    # SUB_FOLDER_NAME = 'Parking/'

    SUB_SUB_FOLDER_NAME = 'Driving backward_'
    # SUB_SUB_FOLDER_NAME = 'Driving forward_'
    # SUB_SUB_FOLDER_NAME = 'Eye Tracking_'

    sub_sub_folder_ind_list = [1]

    # SUB_SUB_FOLDER_IND = '3' # TODO: change the subfolder ind

    # IMAGE_FOLDER = 'images' + str(4) + '/'

    # SOURCE_CAMERA_BAG_FOLDER = "/home/iac_user/post-process/Post-SubjectLL/Baseline/only_driving/17-10-23_10-50-24/" + IMAGE_FOLDER + SUB_FOLDER_NAME + SUB_SUB_FOLDER_NAME +SUB_SUB_FOLDER_IND

    # SAVE_FOLDER_FOR_CAMERA_IMAGES = '/home/iac_user/post-process/1017test/Driving/' + SUB_FOLDER_NAME + SUB_SUB_FOLDER_NAME + SUB_SUB_FOLDER_IND

    for i in range(len(sub_sub_folder_ind_list)):

        SUB_SUB_FOLDER_IND = str(sub_sub_folder_ind_list[i])

        image_ind_list = [1,2,3,4]

        for j in range(len(image_ind_list)):
            IMAGE_FOLDER = 'images' + str(image_ind_list[j]) + '/'

            SOURCE_CAMERA_BAG_FOLDER = "/home/iac_user/post-process/Post-SubjectLL/Baseline/only_driving/17-10-23_10-50-24/images" + IMAGE_FOLDER + SUB_FOLDER_NAME + SUB_SUB_FOLDER_NAME +SUB_SUB_FOLDER_IND

            SAVE_FOLDER_FOR_CAMERA_IMAGES = '/home/iac_user/post-process/1017test/Driving/' + SUB_FOLDER_NAME + SUB_SUB_FOLDER_NAME + SUB_SUB_FOLDER_IND
            
            camera_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/img" + str(image_ind_list[j])

            camera_topic_str = '/camera' + str(image_ind_list[j])+ '/usb_cam'+str(image_ind_list[j]) +'/image_raw/compressed' 
            camera_topic = [camera_topic_str]

            camera_input_folder = SOURCE_CAMERA_BAG_FOLDER

            for (root, dirs, files) in os.walk(camera_input_folder):
                for file in sorted(files):
                    print(file)
                    extract(root, file, camera_topic, camera_output_folder, True)

            print_str = "Image conversion complete for " + camera_output_folder
            print(print_str)
