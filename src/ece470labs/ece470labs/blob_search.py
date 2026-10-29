#!/usr/bin/env python

import cv2
import numpy as np
import math

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = math.atan2(-5,180)
beta = 665
o_r = 240
o_c = 320
tx = (309-o_r)/beta
ty = (145-o_c)/beta

#309, 145   
#489.140

# Function that converts image coord to world coord
def IMG2W(col, row):

    #intrinsic
    x_c = (row-o_r)/beta
    y_c = (col-o_c)/beta

    #extrinsic

    cam = np.array([[x_c], [y_c]])
    t = np.array([[tx], [ty]])
    R = np.array(
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)]
    )
    world = cam @ R + t
    x_w = world[0]
    y_w = world[1]
    return x_w, y_w


# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================
    
    # Threshold
    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 200 # Change to 200 for Orange

    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.75
    params.maxCircularity = 0.8

    # Filter by Inerita
    params.filterByInertia = True
    params.minInertiaRatio = 0.5
    params.maxInertiaRatio = 1.0

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.5

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    # lower = (110,50,50)     # blue lower
    # upper = (130,255,255)   # blue upper
    # lower = (40,50,50)     # green lower
    # upper = (70,255,255)   # green upper
    lower = (5,50,50)     # orange lower
    upper = (15,255,255)   # orange upper

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, outImage=None, flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)
    if len(blob_image_center) > 0:
        r = blob_image_center[0][0]
        c = blob_image_center[0][1]
        IMG2W(r,c)
    
    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
