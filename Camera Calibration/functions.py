import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def display_image(img, opencv, verbose):

    # Print image details
    if verbose:
        print('')
        print(img.shape)
        if opencv:
            # Press any key to exit
            print("\nPress any key to exit, do not click on the 'x' icon")

    # Display the image
    if not opencv:
        plt.imshow(img)
        plt.show()

    else:
        cv2.imshow('img', img)

        cv2.waitKey(0)
        cv2.destroyAllWindows()


def calibration(source_img, pts, opencv):

    #convert the source img into grayscale
    if not opencv:
        gray = cv2.cvtColor(source_img, cv2.COLOR_RGB2GRAY)
    else:
        gray = cv2.cvtColor(source_img, cv2.COLOR_BGR2GRAY)
    print(gray.shape)
    # Find the chessboard corner
    ret, corners = cv2.findChessboardCorners(gray, pts, None)

    # If found, draw corners
    if ret:

        # Draw and display the corners
        cv2.drawChessboardCorners(source_img, pts, corners, ret)

        return source_img
    else:
        return None


def full_calibration(images, pts, opencv, verbose):

    # Create arrays to store object points and image points from all the images
    obj_pts = []    # 3D
    img_pts = []    # 2D

    # Prepare object points
    objp = np.zeros((pts[0] * pts[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pts[0], 0:pts[1]].T.reshape(-1, 2)     # x, y coordinate


    # Load all the image
    for each_image in images:

        if verbose:
            print(each_image, 'calibrated')

        img = cv2.imread(each_image)

        # Convert to grayscale
        if not opencv:
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        else:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corner's coordinate
        ret, corners = cv2.findChessboardCorners(gray, pts, None)

        # If found, perform the code below
        if ret:
            # Add image points
            img_pts.append(corners)

            # Add the object points
            obj_pts.append(objp)

    ret, cam_mtx, dist, r_vecs, t_vecs = cv2.calibrateCamera(obj_pts, img_pts, img.shape[:2], None, None)

    return ret, cam_mtx, dist, r_vecs, t_vecs
