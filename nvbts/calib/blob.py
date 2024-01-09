import json
import cv2

import numpy as np

# import rosbag
# import dask.array as da
import numpy as np
# import rospy

import matplotlib.pyplot as plt

def nothing(x):

    pass

# bag = rosbag.Bag('softening_15112023_1130.bag')

# def imgmsg_to_cv2(img_msg, n_channel=3):
#     dtype = np.uint8
#     if n_channel==1:
#         return np.ndarray(shape=(img_msg.height, img_msg.width),
#                                 dtype=dtype, buffer=img_msg.data)
#     else:
#         return np.ndarray(shape=(img_msg.height, img_msg.width, n_channel),
#                                 dtype=dtype, buffer=img_msg.data)



# from cv_bridge import CvBridge
# bridge = CvBridge()

# events = []

# for topic, msg, t in bag.read_messages(topics=['/dvs/image_raw']):
#     if topic == '/dvs/image_raw':
#         cv_img = imgmsg_to_cv2(msg)
#         cv2.imshow('image', cv_img)
#         k = cv2.waitKey(0)

#         if k == ord('s'):
#             break
#         elif k == ord('n'):
#             continue


center = [172, 112] #(162, 123)
roi_r = 80
frame = cv2.imread('image.png')
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
mask = np.zeros(frame.shape[:2], np.uint8)
mask[120:354, 140:430] = 1
# masked_image = cv2.bitwise_and(frame, mask=mask)
masked_img = gray * mask
gray = masked_img
# gray = cv2.GaussianBlur(gray, (3,3), 5)
# thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 35, 15)
# k_3 = np.ones((3,3),np.uint8)
# k_5 = np.ones((5,5),np.uint8)
# thresh = cv2.erode(thresh,k_3,iterations = 1)
# thresh = cv2.dilate(thresh,k_5,iterations = 1)
# thresh = cv2.erode(thresh, k_3,iterations = 1)

# mask = np.zeros_like(frame)
# mask = cv2.circle(mask, center, roi_r, (255,255,255), -1)

# img = cv2.bitwise_and(frame, mask)
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Create a window

cv2.namedWindow('Adaptive Thresholding and Blob Detection')



# Create trackbars for thresholding

cv2.createTrackbar('BlockSize', 'Adaptive Thresholding and Blob Detection', 15, 50, nothing)

cv2.createTrackbar('C', 'Adaptive Thresholding and Blob Detection', 50, 50, nothing)



# Create trackbars for blob detection

cv2.createTrackbar('minArea', 'Adaptive Thresholding and Blob Detection', 25, 200, nothing)

cv2.createTrackbar('maxArea', 'Adaptive Thresholding and Blob Detection', 200, 1000, nothing)

cv2.createTrackbar('minCircularity', 'Adaptive Thresholding and Blob Detection', 3, 10, nothing)

cv2.createTrackbar('minConvexity', 'Adaptive Thresholding and Blob Detection', 1, 100, nothing)

cv2.createTrackbar('mindist', 'Adaptive Thresholding and Blob Detection', 4, 20, nothing)

cv2.createTrackbar('minInertiaRation', 'Adaptive Thresholding and Blob Detection', 171, 10000, nothing)

cv2.createTrackbar('maxInertiaRation', 'Adaptive Thresholding and Blob Detection', 1000, 10000, nothing)


# Read the image from file

# frame = cv2.imread('/home/mohamadhalwani/VBTS/src/DepthEstimation/scripts/M2_image_undistorted.png')



while True:

    # Convert the frame to grayscale


    # Get current positions of trackbars for thresholding

    blockSize = cv2.getTrackbarPos('BlockSize', 'Adaptive Thresholding and Blob Detection')

    C = cv2.getTrackbarPos('C', 'Adaptive Thresholding and Blob Detection')


    # Make sure blockSize is odd

    if blockSize % 2 == 0:

        blockSize += 1



    # Apply adaptive thresholding

    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, blockSize, C)


    # Blob detection parameters

    params = cv2.SimpleBlobDetector_Params()

    params.minDistBetweenBlobs = cv2.getTrackbarPos('mindist', 'Adaptive Thresholding and Blob Detection')

    params.minThreshold = 60

    params.maxThreshold = 255

    params.filterByArea = True

    params.minArea = cv2.getTrackbarPos('minArea', 'Adaptive Thresholding and Blob Detection')

    params.maxArea = cv2.getTrackbarPos('maxArea', 'Adaptive Thresholding and Blob Detection')

    params.filterByCircularity = True

    params.minCircularity = cv2.getTrackbarPos('minCircularity', 'Adaptive Thresholding and Blob Detection') / 10.0

    params.filterByConvexity = True

    params.minConvexity = cv2.getTrackbarPos('minConvexity', 'Adaptive Thresholding and Blob Detection') / 100.0

    params.filterByInertia = True

    params.minInertiaRatio = cv2.getTrackbarPos('minInertiaRation', 'Adaptive Thresholding and Blob Detection') / 1000.0

    params.maxInertiaRatio = cv2.getTrackbarPos('maxInertiaRation', 'Adaptive Thresholding and Blob Detection') / 1000.0



    # Create blob detector

    detector = cv2.SimpleBlobDetector_create(params)


    # Detect blobs

    keypoints = detector.detect(thresh)


    # Draw keypoints

    im_with_keypoints = cv2.drawKeypoints(thresh, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


    # Display

    cv2.imshow('Adaptive Thresholding and Blob Detection', im_with_keypoints)


    if cv2.waitKey(1) & 0xFF == ord('q'):

        # Get the final trackbar positions

        blockSize = cv2.getTrackbarPos('BlockSize', 'Adaptive Thresholding and Blob Detection')

        C = cv2.getTrackbarPos('C', 'Adaptive Thresholding and Blob Detection')

        params.minArea = cv2.getTrackbarPos('minArea', 'Adaptive Thresholding and Blob Detection')

        params.maxArea = cv2.getTrackbarPos('maxArea', 'Adaptive Thresholding and Blob Detection')

        #params.minCircularity = cv2.getTrackbarPos('minCircularity', 'Adaptive Thresholding and Blob Detection')


        print(params.minDistBetweenBlobs)

        print(params.minThreshold)

        print(params.maxThreshold)



        print(params.minArea)

        print(params.maxArea)



        print(params.minCircularity)

        print(params.minConvexity)

        print(params.minInertiaRatio)

        print(params.maxInertiaRatio)
        
        params_dict = {}

        for attr in dir(params):
            if not attr.startswith('__'):
                params_dict[attr] = getattr(params, attr)
        params_dict.update({"C": C, "blockSize": blockSize})
        
        with open('params.json', 'w') as f:
            
            json.dump(params_dict, fp=f, indent=4)


        break



# thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, blockSize, C)


# detector = cv2.SimpleBlobDetector_create(params)



# # Detect blobs

# keypoints = detector.detect(thresh)



# # Draw keypoints

# im_with_keypoints = cv2.drawKeypoints(thresh, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)



# # Display

# cv2.imshow('Adaptive Thresholding and Blob Detection', im_with_keypoints)

# cv2.waitKey(0)



cv2.destroyAllWindows()
