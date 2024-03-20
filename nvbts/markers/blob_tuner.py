import json
import cv2
import numpy as np

import matplotlib.pyplot as plt

def nothing(x):
    pass


frame = cv2.imread('/home/hussain/Desktop/final_resting.png')


cv2.namedWindow('Adaptive Thresholding and Blob Detection')

# Create trackbars for blob detection

cv2.createTrackbar('minArea', 'Adaptive Thresholding and Blob Detection', 0, 200, nothing)
cv2.createTrackbar('maxArea', 'Adaptive Thresholding and Blob Detection', 0, 1000, nothing)
cv2.createTrackbar('minCircularity', 'Adaptive Thresholding and Blob Detection', 0, 10, nothing)
cv2.createTrackbar('minConvexity', 'Adaptive Thresholding and Blob Detection', 0, 100, nothing)
cv2.createTrackbar('mindist', 'Adaptive Thresholding and Blob Detection', 1, 20, nothing)
cv2.createTrackbar('minInertiaRation', 'Adaptive Thresholding and Blob Detection', 0, 10000, nothing)
cv2.createTrackbar('maxInertiaRation', 'Adaptive Thresholding and Blob Detection', 0, 10000, nothing)

# preprosessing params
size = frame.shape[:2]
cv2.createTrackbar('center_x', 'Adaptive Thresholding and Blob Detection', 0, size[1], nothing)
cv2.createTrackbar('center_y', 'Adaptive Thresholding and Blob Detection', 0, size[0], nothing)
cv2.createTrackbar('radius', 'Adaptive Thresholding and Blob Detection', 0, 500, nothing)

cv2.createTrackbar('BlockSize', 'Adaptive Thresholding and Blob Detection', 15, 50, nothing)
cv2.createTrackbar('C', 'Adaptive Thresholding and Blob Detection', 50, 50, nothing)
cv2.createTrackbar('bilateral', 'Adaptive Thresholding and Blob Detection', 0, 1, nothing)
cv2.createTrackbar('GaussianBlur', 'Adaptive Thresholding and Blob Detection', 0, 1, nothing)
cv2.createTrackbar('clahe', 'Adaptive Thresholding and Blob Detection', 0, 1, nothing)

cv2.createTrackbar('invert', 'Adaptive Thresholding and Blob Detection', 0, 1, nothing)

cv2.createTrackbar('Opening', 'Adaptive Thresholding and Blob Detection', 0, 5, nothing)
cv2.createTrackbar('Closing', 'Adaptive Thresholding and Blob Detection', 0, 5, nothing)
cv2.createTrackbar('Erosion', 'Adaptive Thresholding and Blob Detection', 0, 5, nothing)
cv2.createTrackbar('Dilation', 'Adaptive Thresholding and Blob Detection', 0, 5, nothing)



while True:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    blockSize = cv2.getTrackbarPos('BlockSize', 'Adaptive Thresholding and Blob Detection')
    C = cv2.getTrackbarPos('C', 'Adaptive Thresholding and Blob Detection')

    if blockSize % 2 == 0:
        blockSize += 1

    center_x = cv2.getTrackbarPos('center_x', 'Adaptive Thresholding and Blob Detection')
    center_y = cv2.getTrackbarPos('center_y', 'Adaptive Thresholding and Blob Detection')
    radius = cv2.getTrackbarPos('radius', 'Adaptive Thresholding and Blob Detection')

    mask = np.zeros(frame.shape[:2], np.uint8)
    mask = cv2.circle(mask, (center_x, center_y), radius, (255, 255, 255), -1, 8, 0)

    gray = cv2.bitwise_and(gray, gray, mask=mask)

    if cv2.getTrackbarPos('GaussianBlur', 'Adaptive Thresholding and Blob Detection') == 1:
        gray = cv2.GaussianBlur(gray, (3,3), 5)

    if cv2.getTrackbarPos('bilateral', 'Adaptive Thresholding and Blob Detection') == 1:
        gray = cv2.bilateralFilter(gray, 5, 75, 75)

    if cv2.getTrackbarPos('clahe', 'Adaptive Thresholding and Blob Detection') == 1:
        clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)

    # Apply adaptive thresholding

    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, blockSize, C)

    if cv2.getTrackbarPos('invert', 'Adaptive Thresholding and Blob Detection') == 1:
        thresh = cv2.bitwise_not(thresh)

    if cv2.getTrackbarPos('Opening', 'Adaptive Thresholding and Blob Detection') > 0:
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (cv2.getTrackbarPos('Opening', 'Adaptive Thresholding and Blob Detection'), cv2.getTrackbarPos('Opening', 'Adaptive Thresholding and Blob Detection')))

    if cv2.getTrackbarPos('Closing', 'Adaptive Thresholding and Blob Detection') > 0:
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, (cv2.getTrackbarPos('Closing', 'Adaptive Thresholding and Blob Detection'), cv2.getTrackbarPos('Closing', 'Adaptive Thresholding and Blob Detection')))

    if cv2.getTrackbarPos('Erosion', 'Adaptive Thresholding and Blob Detection') > 0:
        thresh = cv2.erode(thresh, (cv2.getTrackbarPos('Erosion', 'Adaptive Thresholding and Blob Detection'), cv2.getTrackbarPos('Erosion', 'Adaptive Thresholding and Blob Detection')))

    params = cv2.SimpleBlobDetector_Params()
    params.minDistBetweenBlobs = cv2.getTrackbarPos('mindist', 'Adaptive Thresholding and Blob Detection')
    params.minThreshold = 60
    params.maxThreshold = 255

    params.filterByArea = cv2.getTrackbarPos('minArea', 'Adaptive Thresholding and Blob Detection') > 0

    if params.filterByArea:
        params.minArea = cv2.getTrackbarPos('minArea', 'Adaptive Thresholding and Blob Detection')
        params.maxArea = cv2.getTrackbarPos('maxArea', 'Adaptive Thresholding and Blob Detection')

    params.filterByCircularity = cv2.getTrackbarPos('minCircularity', 'Adaptive Thresholding and Blob Detection') > 0
    if params.filterByCircularity:
        params.minCircularity = cv2.getTrackbarPos('minCircularity', 'Adaptive Thresholding and Blob Detection') / 10.0


    params.filterByConvexity = cv2.getTrackbarPos('minConvexity', 'Adaptive Thresholding and Blob Detection') > 0
    if params.filterByConvexity:
        params.minConvexity = cv2.getTrackbarPos('minConvexity', 'Adaptive Thresholding and Blob Detection') / 100.0

    params.filterByInertia = cv2.getTrackbarPos('minInertiaRation', 'Adaptive Thresholding and Blob Detection') > 0

    if params.filterByInertia:
        params.minInertiaRatio = cv2.getTrackbarPos('minInertiaRation', 'Adaptive Thresholding and Blob Detection') / 1000.0
        params.maxInertiaRatio = cv2.getTrackbarPos('maxInertiaRation', 'Adaptive Thresholding and Blob Detection') / 1000.0

    # Create blob detector

    detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs

    keypoints = detector.detect(thresh)

    # Draw keypoints
    im_with_keypoints = cv2.drawKeypoints(thresh, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # draw center
    im_with_keypoints = cv2.circle(im_with_keypoints, (center_x, center_y), 5, (0, 255, 0), -1)
    # im_with_keypoints = cv2.addText(im_with_keypoints, f"center", (center_x, center_y), "Arial", 1, (0, 255, 0))

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

        preprocess_params = {
            'center_x': center_x,
            'center_y': center_y,
            'radius': radius,
            'BlockSize': blockSize,
            'C': C,
            'bilateral': cv2.getTrackbarPos('bilateral', 'Adaptive Thresholding and Blob Detection'),
            'GaussianBlur': cv2.getTrackbarPos('GaussianBlur', 'Adaptive Thresholding and Blob Detection'),
            'clahe': cv2.getTrackbarPos('clahe', 'Adaptive Thresholding and Blob Detection'),
            'invert': cv2.getTrackbarPos('invert', 'Adaptive Thresholding and Blob Detection'),
            'Opening': cv2.getTrackbarPos('Opening', 'Adaptive Thresholding and Blob Detection'),
            'Closing': cv2.getTrackbarPos('Closing', 'Adaptive Thresholding and Blob Detection'),
            'Erosion': cv2.getTrackbarPos('Erosion', 'Adaptive Thresholding and Blob Detection'),
            'Dilation': cv2.getTrackbarPos('Dilation', 'Adaptive Thresholding and Blob Detection')
        }

        for attr in dir(params):
            if not attr.startswith('__'):
                params_dict[attr] = getattr(params, attr)
        
        with open('nvbts/markers/blob.json', 'w') as f:
            json.dump({
                'preprocess_params': preprocess_params,
                'blob_params': params_dict
            }, f, indent=4)


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
