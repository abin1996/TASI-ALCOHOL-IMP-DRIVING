import os
import rosbag
import cv2
from cv_bridge import CvBridge, CvBridgeError

RECORDING_FOLDER_NAME = '22-09-23_15:25:12'

SOURCE_CAMERA_BAG_FOLDER = "/home/iac_user/DATA_COLLECTION/" + RECORDING_FOLDER_NAME
SAVE_FOLDER_FOR_CAMERA_IMAGES = '/home/iac_user/PROCESSED_DATA_COLLECTION/' + RECORDING_FOLDER_NAME
count = 0
def extract(dirs, filename,topic, output_folder, flipped):
    global count
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


camera1_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/img1"
camera2_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/img2"
camera1_topic = ['/camera1/usb_cam1/image_raw/compressed']
camera2_topic = ['/camera2/usb_cam2/image_raw/compressed']
camera1_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images1"
camera2_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images2"
# camera_list = [camera1_folder, camera2_folder]
for (root, dirs, files) in os.walk(camera1_input_folder):
    for file in sorted(files):
        print(file)
        extract(root, file, camera1_topic, camera1_output_folder, True)
        #break
for (root, dirs, files) in os.walk(camera2_input_folder):
    for file in sorted(files):
        print(file)
        extract(root, file, camera2_topic, camera2_output_folder, True)

camera3_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/img3"
camera4_output_folder = SAVE_FOLDER_FOR_CAMERA_IMAGES + "/img4"
camera3_topic = ['/camera3/usb_cam4/image_raw/compressed']
camera4_topic = ['/camera4/usb_cam4/image_raw/compressed']
camera3_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images3"
camera4_input_folder = SOURCE_CAMERA_BAG_FOLDER + "/images4"
# camera_list = [camera1_folder, camera2_folder]
for (root, dirs, files) in os.walk(camera3_input_folder):
    for file in sorted(files):
        print(file)
        extract(root, file, camera3_topic, camera3_output_folder, True)
        #break
for (root, dirs, files) in os.walk(camera4_input_folder):
    for file in sorted(files):
        print(file)
        extract(root, file, camera4_topic, camera4_output_folder, True)
