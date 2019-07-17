import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def perspect_transform():
    pass


if __name__ == '__main__':

    # Load the image
    img = mpimg.imread('example_grid1.jpg')

    # Obtain the source points to perform perspective transform
    points = np.float32([15, 140], [302, 104], [200, 96], [118, 96])

    # Display the image
    plt.imshow(img)
    plt.show()