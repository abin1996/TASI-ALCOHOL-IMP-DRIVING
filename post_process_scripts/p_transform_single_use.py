import cv2
import numpy as np
import os
import logging

log = logging.getLogger(__name__)
def p_transform(corner_positions, calibration_img, input_folder, output_folder, flip_image):
    '''''''''
    The corner_postions must follow the order: TOP LEFT -----> TOP RIGHT ------> BOTTOM LEFT -----> BOTTOM RIGHT
    
    The coordinates need to be obtained from the paint app
        '''''''''

    if not os.path.exists(input_folder):
        print(f"The input folder '{input_folder}' does not exist. Stopping.")
        return
    #Clear output folder before running
    # if os.path.exists(output_folder):
    #      for file in os.listdir(output_folder):
    #         file_path = os.path.join(output_folder, file)
    #         os.remove(file_path)
    # if not os.path.exists(output_folder):
    #         os.makedirs(output_folder, exist_ok=True)

    # if flip_image:
    # # # Flip the image horizontally
    #     calibration_img = cv2.flip(calibration_img, 1) # Pay attention the command may be FLIP_TOP_BOTTOM to have the sedired orientation
    #     # print('flipped image')
    # # Save the flipped image
    #     filename = os.path.join(output_folder,"flipped_left_cam_cali_img.png")
    #     print(filename)
    #     cv2.imwrite(filename, calibration_img)

    # read the coordinates from the corner_positions input
    pt1_x, pt1_y = corner_positions[0]
    pt2_x, pt2_y = corner_positions[1]
    pt3_x, pt3_y = corner_positions[2]
    pt4_x, pt4_y = corner_positions[3]

    # top left
    cv2.circle(calibration_img, (pt1_x, pt1_y), 5, (0, 0, 255), -1)
    # top right
    cv2.circle(calibration_img, (pt2_x, pt2_y), 5, (0, 0, 255), -1)
    # bottom left
    cv2.circle(calibration_img, (pt3_x, pt3_y), 5, (0, 0, 255), -1)
    # bottom right
    cv2.circle(calibration_img, (pt4_x, pt4_y), 5, (0, 0, 255), -1)

    pts1 = np.float32([[pt1_x, pt1_y], [pt2_x, pt2_y], [pt3_x, pt3_y], [pt4_x, pt4_y]])

    num_trans_hor_pixel = 1450
    num_trans_ver_pixel = 750

    pts2 = np.float32(
        [[0, 0], [num_trans_hor_pixel, 0], [0, num_trans_ver_pixel], [num_trans_hor_pixel, num_trans_ver_pixel]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    result = cv2.warpPerspective(calibration_img, matrix, (num_trans_hor_pixel, num_trans_ver_pixel))

    mat_hor_len = 1455  # unit: mm
    dpp_hor = mat_hor_len / num_trans_hor_pixel
    # print('horizontal dpp:', dpp_hor, 'mm')
    # filename = os.path.join(output_folder,"Transformed_image.jpg")
    filename = "/home/tasi/Transformed_image_left.jpg"
    log.debug(filename)
    cv2.imwrite(filename, result)
    print(filename)
    print('saved transformed image')
    # log.debug('saved transformed image')
    # if not os.path.exists(output_folder):
    #     os.makedirs(output_folder, exist_ok=True)
    # png_files = [f for f in os.listdir(input_folder) if f.lower().endswith(".png")]
    # for png_file in png_files:
    #     input_path = os.path.join(input_folder, png_file)
    #     output_path = os.path.join(output_folder, png_file)
    #
    #     # Read the image
    #     image = cv2.imread(input_path)
    #     if flip_image:
    #         image = cv2.flip(image, 1)
    #     # Apply the perspective transformation
    #     transformed_image = cv2.warpPerspective(image, matrix, (num_trans_hor_pixel, num_trans_ver_pixel))
    #
    #     # Save the transformed image
    #     cv2.imwrite(output_path, transformed_image)
    #
    #     log.debug(f"Transformed and saved: {output_path}")
    #
    # log.info("All files transformed and saved for {}".format(input_folder))
    #

if __name__ == "__main__":

    lane_deviation_parameters = {
        "corner_points": {
            "top_left": [
                24,
                9
            ],
            "top_right": [
                522,
                50
            ],
            "bottom_left": [
                51,
                363
            ],
            "bottom_right": [
                506,
                281
            ]
        }
    }
    top_left = lane_deviation_parameters['corner_points']['top_left']
    top_right = lane_deviation_parameters['corner_points']['top_right']
    bottom_left = lane_deviation_parameters['corner_points']['bottom_left']
    bottom_right = lane_deviation_parameters['corner_points']['bottom_right']
    corner_positions = [top_left, top_right, bottom_left, bottom_right]
    cali_img = cv2.imread("/media/tasi/InternalDrive1/DATA_COLLECTION/ORG_DATA/Subject04/Subject04_cali_img/left_cali_Subject04_flipped.jpg")

    input_folder = "/media/tasi/InternalDrive1/DATA_COLLECTION/ORG_DATA/Subject03/Subejct03_cali_img"
    output_folder = "/media/tasi/InternalDrive1/DATA_COLLECTION/ORG_DATA/Subject04/Subject04_cali_img"

    p_transform(corner_positions, cali_img, input_folder, output_folder,True)