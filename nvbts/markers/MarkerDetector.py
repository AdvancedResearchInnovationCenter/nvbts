import cv2
import numpy as np
import json


class MarkerDetector:

    def __init__(self, blob_tuner_file, camera_intrinsics) -> None:
        with open(blob_tuner_file) as f:
            self.blob_tuner = json.load(f)

        with open(camera_intrinsics) as f:
            self.camera_intrinsics = json.load(f)
            self.camera_matrix = np.array(self.camera_intrinsics['camera_matrix'])
            self.dist_coeffs = np.array(self.camera_intrinsics['distortion_coeffs'])

        preprocess_params = self.blob_tuner['preprocess_params']
        blob_params = self.blob_tuner['blob_params']

        self.center_x = preprocess_params['center_x']
        self.center_y = preprocess_params['center_y']
        self.radius = preprocess_params['radius']
        self.BlockSize = preprocess_params['BlockSize']
        self.C = preprocess_params['C']
        self.bilateral = preprocess_params['bilateral']
        self.GaussianBlur = preprocess_params['GaussianBlur']
        self.clahe = preprocess_params['clahe']
        self.invert = preprocess_params['invert']
        self.Opening = preprocess_params['Opening']
        self.Closing = preprocess_params['Closing']
        self.Erosion = preprocess_params['Erosion']
        self.Dilation = preprocess_params['Dilation']

        self.minDistBetweenBlobs = blob_params['minDistBetweenBlobs']
        self.minThreshold = blob_params['minThreshold']
        self.maxThreshold = blob_params['maxThreshold']
        self.minArea = blob_params['minArea']
        self.maxArea = blob_params['maxArea']
        self.minCircularity = blob_params['minCircularity']
        self.minConvexity = blob_params['minConvexity']
        self.minInertiaRatio = blob_params['minInertiaRatio']
        self.maxInertiaRatio = blob_params['maxInertiaRatio']

    def apply(self, img, draw_keypoints=False):

        #undistort image
        img = cv2.undistort(img, self.camera_matrix, self.dist_coeffs)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if self.BlockSize % 2 == 0:
            self.BlockSize += 1

        mask = np.zeros(img.shape[:2], np.uint8)
        mask = cv2.circle(mask, (self.center_x, self.center_y), self.radius, (255, 255, 255), -1, 8, 0)

        gray = cv2.bitwise_and(gray, gray, mask=mask)

        if self.GaussianBlur == 1:
            gray = cv2.GaussianBlur(gray, (3,3), 5)

        if self.bilateral == 1:
            gray = cv2.bilateralFilter(gray, 5, 75, 75)

        if self.clahe == 1:
            clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8,8))
            gray = clahe.apply(gray)

        # Apply adaptive thresholding
            
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, self.BlockSize, self.C)

        if self.invert == 1:
            thresh = cv2.bitwise_not(thresh)

        if self.Opening > 0:
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (self.Opening, self.Opening))

        if self.Closing > 0:
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, (self.Closing, self.Closing))

        if self.Erosion > 0:
            thresh = cv2.erode(thresh, (self.Erosion, self.Erosion))

        # Detect blobs.
        # params = cv2.SimpleBlobDetector_Params()
        # params.minDistBetweenBlobs = cv2.getTrackbarPos('mindist', 'Adaptive Thresholding and Blob Detection')
        # params.minThreshold = 60
        # params.maxThreshold = 255

        # params.filterByArea = cv2.getTrackbarPos('minArea', 'Adaptive Thresholding and Blob Detection') > 0

        # if params.filterByArea:
        #     params.minArea = cv2.getTrackbarPos('minArea', 'Adaptive Thresholding and Blob Detection')
        #     params.maxArea = cv2.getTrackbarPos('maxArea', 'Adaptive Thresholding and Blob Detection')

        # params.filterByCircularity = cv2.getTrackbarPos('minCircularity', 'Adaptive Thresholding and Blob Detection') > 0
        # if params.filterByCircularity:
        #     params.minCircularity = cv2.getTrackbarPos('minCircularity', 'Adaptive Thresholding and Blob Detection') / 10.0


        # params.filterByConvexity = cv2.getTrackbarPos('minConvexity', 'Adaptive Thresholding and Blob Detection') > 0
        # if params.filterByConvexity:
        #     params.minConvexity = cv2.getTrackbarPos('minConvexity', 'Adaptive Thresholding and Blob Detection') / 100.0

        # params.filterByInertia = cv2.getTrackbarPos('minInertiaRation', 'Adaptive Thresholding and Blob Detection') > 0

        # if params.filterByInertia:
        #     params.minInertiaRatio = cv2.getTrackbarPos('minInertiaRation', 'Adaptive Thresholding and Blob Detection') / 1000.0
        #     params.maxInertiaRatio = cv2.getTrackbarPos('maxInertiaRation', 'Adaptive Thresholding and Blob Detection') / 1000.0

        # # Create blob detector

        # detector = cv2.SimpleBlobDetector_create(params)
        # # Detect blobs

        # keypoints = detector.detect(thresh)

        # # Draw keypoints
        # im_with_keypoints = cv2.drawKeypoints(thresh, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # # draw center
    
        # im_with_keypoints = cv2.circle(im_with_keypoints, (center_x, center_y), 5, (0, 255, 0), -1)


        params = cv2.SimpleBlobDetector_Params()
        params.minDistBetweenBlobs = self.minDistBetweenBlobs
        params.minThreshold = self.minThreshold
        params.maxThreshold = self.maxThreshold

        params.filterByArea = self.minArea > 0
        if params.filterByArea:
            params.minArea = self.minArea
            params.maxArea = self.maxArea

        params.filterByCircularity = self.minCircularity > 0
        if params.filterByCircularity:
            params.minCircularity = self.minCircularity / 10.0

        params.filterByConvexity = self.minConvexity > 0
        if params.filterByConvexity:
            params.minConvexity = self.minConvexity / 100.0

        params.filterByInertia = self.minInertiaRatio > 0
        if params.filterByInertia:
            params.minInertiaRatio = self.minInertiaRatio / 1000.0
            params.maxInertiaRatio = self.maxInertiaRatio / 1000.0

        # Create blob detector
        detector = cv2.SimpleBlobDetector_create(params)
        # Detect blobs
        keypoints = detector.detect(thresh)

        if draw_keypoints:
            # Draw keypoints
            im_with_keypoints = cv2.drawKeypoints(thresh, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # draw center
            im_with_keypoints = cv2.circle(im_with_keypoints, (self.center_x, self.center_y), 5, (0, 255, 0), -1)

            return im_with_keypoints, np.array([k.pt for k in keypoints])

        else:
            return np.array([k.pt for k in keypoints])



