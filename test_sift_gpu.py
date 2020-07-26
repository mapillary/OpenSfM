import cv2
import matplotlib.pyplot as plt
from opensfm.sift_gpu import SiftGpu
import numpy as np


def draw_matches(img1, kp1, img2, kp2, matches, color=None):
    """Draws lines between matching keypoints of two images.  
    Keypoints not in a matching pair are not drawn.
    Places the images side by side in a new image and draws circles 
    around each keypoint, with line segments connecting matching pairs.
    You can tweak the r, thickness, and figsize values as needed.
    Args:
        img1: An openCV image ndarray in a grayscale or color format.
        kp1: A list of cv2.KeyPoint objects for img1.
        img2: An openCV image ndarray of the same format and with the same 
        element type as img1.
        kp2: A list of cv2.KeyPoint objects for img2.
        matches: A list of DMatch objects whose trainIdx attribute refers to 
        img1 keypoints and whose queryIdx attribute refers to img2 keypoints.
        color: The color of the circles and connecting lines drawn on the images.  
        A 3-tuple for color images, a scalar for grayscale images.  If None, these
        values are randomly generated.  
    """
    # We're drawing them side by side.  Get dimensions accordingly.
    # Handle both color and grayscale images.
    if len(img1.shape) == 3:
        new_shape = (max(img1.shape[0], img2.shape[0]), img1.shape[1] + img2.shape[1], img1.shape[2])
    elif len(img1.shape) == 2:
        new_shape = (max(img1.shape[0], img2.shape[0]), img1.shape[1] + img2.shape[1])
    new_img = np.zeros(new_shape, type(img1.flat[0]))
    # Place images onto the new image.
    new_img[0:img1.shape[0], 0:img1.shape[1]] = img1
    new_img[0:img2.shape[0], img1.shape[1]:img1.shape[1] + img2.shape[1]] = img2

    # Draw lines between matches.  Make sure to offset kp coords in second image appropriately.
    r = 15
    thickness = 2
    if color:
        c = color
    for m in matches:
        # Generate random color for RGB/BGR and grayscale images as needed.
        if not color:
            c = np.random.randint(0, 256, 3) if len(img1.shape) == 3 else np.random.randint(0, 256)
        # So the keypoint locs are stored as a tuple of floats.  cv2.line(), like most other things,
        # wants locs as a tuple of ints.
        end1 = tuple(np.round(kp1[m[0]].pt).astype(int))
        end2 = tuple(np.round(kp2[m[1]].pt).astype(int) + np.array([img1.shape[1], 0]))
        cv2.line(new_img, end1, end2, c, thickness)
        cv2.circle(new_img, end1, r, c, thickness)
        cv2.circle(new_img, end2, r, c, thickness)

    plt.figure(figsize=(15, 15))
    plt.imshow(new_img)
    plt.show()


if __name__ == '__main__':
    # read images
    img1 = cv2.imread('data_1/berlin/images/01.jpg')  # load img1
    img2 = cv2.imread('data_1/berlin/images/02.jpg')  # load img1
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)  # change to rgb
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)  # change to rgb
    # sift
    sift = SiftGpu(img1)  # initialize the sift_gpu class

    keypoints_1 = sift.detect_image(img1)  # detect features
    idx = np.where(np.sum(keypoints_1.desc, 1) != 0)  # remove zero descriptors from keypoints
    keypoints_1 = keypoints_1[idx]

    keypoints_2 = sift.detect_image(img2)  # detect features
    idx = np.where(np.sum(keypoints_2.desc, 1) != 0)  # remove zero descriptors from keypoints
    keypoints_2 = keypoints_2[idx]

    # feature matching
    matches = sift.match_images(keypoints_1, keypoints_2)  # matching between 2 keypoints

    # transform into cv2 keypoints
    cv_kp1 = [cv2.KeyPoint(x=keypoints_1.x[i], y=keypoints_1.y[i], _size=20) for i in range(len(keypoints_1.x))]
    cv_kp2 = [cv2.KeyPoint(x=keypoints_2.x[i], y=keypoints_2.y[i], _size=20) for i in range(len(keypoints_2.x))]

    # draw the N first matches
    N = 50
    # draw_matches(img1, cv_kp1, img2, cv_kp2, matches[:N])

    # draw the features on img 1
    img3 = np.array([])
    img3 = cv2.drawKeypoints(img2, cv_kp2, img3, color=(0, 0, 255))
    plt.imshow(img3), plt.show()
