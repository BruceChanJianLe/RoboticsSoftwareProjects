import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from functions import *
import glob


if __name__ == '__main__':

    # Use openCV or matplotlib
    opencv = False
    file_path_training = 'calibration_wide/GOPR00*.jpg'
    #file_path_testing = 'calibration_wide/test_image.jpg'
    file_path_testing = 'calibration_wide/test_image.jpg'
    verbose = True

    # Load the testing image
    if not opencv:
        img = mpimg.imread(file_path_testing)

    else:
        img = cv2.imread(file_path_testing)

    # Load the training images
    images = glob.glob(file_path_training)

    # Find the corner of chessboard for calibration
    corner_points = (8, 6)      # 8 points in x direction and 6 in y, image coordinate
    ret, cam_mtx, dist, rotation, translation = full_calibration(images, corner_points, opencv, verbose)

    # Use the calibration parameters
    undistorted = cv2.undistort(img, cam_mtx, dist, None, cam_mtx)

    # Display the image
    display_image(np.hstack([undistorted, img]), opencv, verbose=True)

