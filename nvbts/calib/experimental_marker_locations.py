#!/usr/bin/python3


import json
import numpy as np
import cv2

def roundup(x):
    n = 9
    return np.ceil(x/n) * n

def experimental_marker_array():
    camera_matrix = np.array([[438.59222618,   0.,         323.89686632],
                            [  0.  ,       439.81991094, 240.11146683],
                            [0., 0., 1.]])
    distortion_coeffs = np.array([0.11232303654662454, -0.47277989637474027, -6.726553087628563e-05, 0.0005557680260483787, 0.5754486589853096])

    # blob detection from a still image
    img = cv2.imread("image.png")
    img = cv2.undistort(img, camera_matrix, distortion_coeffs)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = np.zeros(img.shape[:2], np.uint8)
    mask[120:354, 140:430] = 1
    # masked_image = cv2.bitwise_and(frame, mask=mask)
    gray = gray * mask

    rect = cv2.rectangle(img, (140,120), (430,354), (255,0,0))

    params = cv2.SimpleBlobDetector_Params()

    with open('params.json', 'r') as file:

        params_dict = json.load(file)

    for key, value in params_dict.items():
        if key == 'C' or key == 'blockSize':
            continue
        setattr(params, key, value)

    blockSize = params_dict['blockSize']
    C = params_dict['C']

    detector = cv2.SimpleBlobDetector_create(params)
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, blockSize, C)

    keypoints = detector.detect(thresh)

    im_with_keypoints = cv2.drawKeypoints(thresh, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


    rois_pt1 = [(140, 120),(170, 120),(200, 120),(230, 120),(260, 120),(300, 120),(330, 120),(360, 120),(390, 120), (420, 120)]
    # rois_pt2 = [(170, 335), (200, 335), (230, 335), (260, 335), (300, 335), (330, 335), (360, 335), (390, 335), (420, 335)]
    col_boundaries = np.array(rois_pt1)[:, 0]

    img = im_with_keypoints


    cols = []
    for i in range(0, len(keypoints), 1):
        for col in  range(10):
            if col_boundaries[col-1] <= keypoints[i].pt[0] and keypoints[i].pt[0] < col_boundaries[col]:
                cols.append(col)
                
    cols = np.array(cols)
            
    arr_markers = np.zeros((7, 9, 2))

    keypoints = np.array([kp.pt for kp in keypoints])

    for col in range(1, 10):
        keypoints_at_col = keypoints[col == cols]
        arr_markers[:, col-1, :] = keypoints_at_col[np.argsort(keypoints_at_col[:, 1])]
        
    return arr_markers



# idx_x = np.argsort(rounded_x)

# rounded_x = rounded_x[idx_x]
# rounded_y = rounded_y[idx_x]

# keypoints_x = keypoints_x[idx_x]
# keypoints_y = keypoints_y[idx_x]
# # for i in range (len(keypoints_x)):
# #     print(f'{keypoints_x[i]} : {rounded_x[i]}', f'{keypoints_y[i]} : {rounded_y[i]}')
    # print()

# import matplotlib.pyplot as plt

# plt.imshow(img)

# plt.scatter(*arr_markers.reshape(63, 2).T, c='r')#, alpha=np.arange(1, 64) / 128 + 1/2)
# plt.show()

# cv2.imshow("ddd", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()