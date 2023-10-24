# Debug

import os

SOURCE_FOLDER = '/home/iac_user/post-process/Post-SubjectLL/Baseline/only_driving/17-10-23_10-50-24'


sub_folder_name = 'Driving backward'

SUB_FOLDER_NAME = sub_folder_name + '/'

SUB_SUB_FOLDER_NAME = sub_folder_name +'_'

image_ind_list = [1,2,3,4]

for j in range(len(image_ind_list)):
    IMAGE_FOLDER = 'images' + str(image_ind_list[j]) + '/'

    SOURCE_CAMERA_BAG_FOLDER = SOURCE_FOLDER +'/'+ IMAGE_FOLDER + sub_folder_name

    num_sub_sub_folder = len([folder for folder in os.listdir(SOURCE_CAMERA_BAG_FOLDER) if os.path.isdir(os.path.join(SOURCE_CAMERA_BAG_FOLDER, folder))])

print(num_sub_sub_folder)