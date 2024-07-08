
import cv2
import numpy as np
import matplotlib.pyplot as plt

def manual_blob(frame, gblur=(7, 3), roi_x=None, roi_y=None, c=None, r=None):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if roi_x is None:
        roi_x = [0, frame.shape[0]]
    if roi_y is None:
        roi_y = [0, frame.shape[1]]

    frame = frame[roi_x[0]:roi_x[1], roi_y[0]:roi_y[1]]
    shape = np.array(frame.shape)
    #upscale
    # frame = cv2.resize(frame, 3*shape, interpolation=cv2.INTER_LINEAR)
    frame_ = frame.copy()
    # frame_ = cv2.cvtColor(frame_, cv2.COLOR_GRAY2RGB)
    frame = cv2.GaussianBlur(frame, (gblur[0], gblur[0]), gblur[1])

    thresh = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 41, 1)
    thresh = cv2.bitwise_not(thresh)

    #morphological operations
    k3 = np.ones((3, 3), np.uint8)
    thresh = cv2.dilate(thresh, k3)
    thresh = cv2.erode(thresh, k3)

    # circle = cv2.circle(thresh, (c[0], c[1]), r, (0,0,0), -1)
    # thresh = cv2.bitwise_not(circle)

    #roi circle radius r at c

    mask = np.zeros_like(thresh)
    cv2.circle(mask, (c[0], c[1]), r, (255,255,255), -1)
    thresh = cv2.bitwise_and(thresh, mask)

    # thresh = cv2.erode(thresh, kernel, iterations=1)
    # thresh = cv2.dilate(thresh, kernel, iterations=1)
    frame = thresh


    #apply blob detection and imshow 

    blob_params = cv2.SimpleBlobDetector_Params()
    blob_params.filterByArea = False
    blob_params.minArea = 100
    blob_params.maxArea = 2000
    blob_params.filterByCircularity = False
    blob_params.minCircularity = 0.1
    blob_params.filterByConvexity = False
    blob_params.minConvexity = 0.87
    blob_params.filterByInertia = False
    blob_params.minInertiaRatio = 0.01
    detector = cv2.SimpleBlobDetector_create(blob_params)
    keypoints = detector.detect(frame)
    global kp
    kp = [kp.pt for kp in keypoints]

    kp = np.array(kp)
    
    fig = plt.figure(figsize=(10, 10))
    plt.imshow(thresh, cmap='gray')

    # plt.scatter(kp[:, 0], kp[:, 1], c='r', s=20)

    #if left mouse button is pressed, remove the closest keypoint. If right mouse button is pressed, add a keypoint.

    def onclick(event):
        global kp
        if event.button == 1:
            dist = np.linalg.norm(kp - np.array([event.xdata, event.ydata]), axis=1)
            idx = np.argmin(dist)
            kp_ = np.delete(kp, idx, axis=0)
        elif event.button == 3:
            kp_ = np.vstack([kp, [event.xdata, event.ydata]])
        elif event.button == 2:
            #save keypoints
            # kp = kp_
            # np.save(f'/home/aric/piezo/sliding/markers/kp_{i}.npy', kp)
            plt.clf()
            return
        kp = kp_

        print(kp.shape)
        plt.clf()
        plt.imshow(frame_, cmap='gray')
        plt.scatter(kp[:, 0], kp[:, 1], c='r', s=20)
        plt.title(f'frame, n_markers: {kp.shape[0]}')
        plt.show()

    fig.canvas.mpl_connect('button_press_event', onclick)


    plt.show()

    return kp


if __name__ == '__main__':
    img = cv2.imread('/home/aric/Desktop/image.png')
    roi_y = [87,240]
    roi_x = [45,205]
    c = [162, 126]
    r = 80
    kp = manual_blob(img, c=c, r=r, gblur=(7, 3))
    print(kp.shape)

    with open('kp.npy', 'wb') as f:
        np.save(f, kp)
