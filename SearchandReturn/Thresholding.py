import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2


# Define a function to perform a color threshold
def color_thresh(self, lower_thresh=np.array([0, 0, 0]), upper_thresh=np.array([0, 0, 0])):

    # Create a mask for the image
    mask = cv2.inRange(self, lower_thresh, upper_thresh)

    return mask


if __name__ == '__main__':

    # Load the image
    img = mpimg.imread('sample.jpg')

    # Print the shape of the image
    print(img.shape)

    # Obtain a deep copy of the image
    r_img_copy = img.copy()
    g_img_copy = img.copy()
    b_img_copy = img.copy()

    # Split the image into RGB but still showing it in RGB, hence, have to make the other channel 0
    r_img_copy[:, :, [1, 2]] = 0
    g_img_copy[:, :, [0, 2]] = 0
    b_img_copy[:, :, [0, 1]] = 0

    # Display the image
    plt.imshow(np.hstack([r_img_copy, g_img_copy, b_img_copy]))
    plt.show()

    # Obtain a deep copy of the image
    img_copy = img.copy()

    # Input the value to perform thresholding
    lower = np.array([170, 170, 150])
    upper = np.array([255, 255, 255])

    mask = color_thresh(img_copy, lower, upper)

    # Use the mask to create the binary image
    img_copy[mask == 0] = [0, 0, 0]
    img_copy[mask != 0] = [255, 255, 255]

    # Display the image
    plt.imshow(img_copy)
    plt.show()
