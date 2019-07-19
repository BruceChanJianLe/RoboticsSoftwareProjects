import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def perspect_transform(img, pts, dst):

    # Obtain the transformation matrix
    m = cv2.getPerspectiveTransform(pts, dst)

    # Warp the image
    warped = cv2.warpPerspective(img, m, (img.shape[1], img.shape[0]))

    return m, warped


def color_thresh(self, lower_thresh=np.array([0, 0, 0]), upper_thresh=np.array([0, 0, 0])):

    # Create a mask for the image
    mask = cv2.inRange(self, lower_thresh, upper_thresh)

    return mask


if __name__ == '__main__':

    # Load the image
    img = mpimg.imread('example_grid1.jpg')

    # Obtain the source points to perform perspective transform
    source = np.float32([[15, 140], [302, 140], [200, 96], [118, 96]])

    # Obtain a deep copy of the image
    img_copy = img.copy()

    # Parameters for the function, transforming to a 10 by 10 area
    dst_size = 5
    bottom_offset = 6
    destination = np.float32([[img.shape[1] / 2 - dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1] / 2 + dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1] / 2 + dst_size, img.shape[0] - 2 * dst_size - bottom_offset],
                              [img.shape[1] / 2 - dst_size, img.shape[0] - 2 * dst_size - bottom_offset],
                              ])

    # Use the perspect_transform function
    M, output = perspect_transform(img_copy, pts=source, dst=destination)

    # Save the image
    new = output.copy()
    new = cv2.cvtColor(new, cv2.COLOR_RGB2BGR)
    cv2.imwrite('After_Transform.jpg', new)

    # Print info
    print(f'{destination}\n {M}')

    # Display the image
    plt.imshow(output)
    plt.show()
