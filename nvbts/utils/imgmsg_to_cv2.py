
import numpy as np
def imgmsg_to_cv2(img_msg, n_channel=3):
    dtype = np.uint8
    if n_channel==1:
        return np.ndarray(shape=(img_msg.height, img_msg.width),
                                dtype=dtype, buffer=img_msg.data)
    else:
        return np.ndarray(shape=(img_msg.height, img_msg.width, n_channel),
                                dtype=dtype, buffer=img_msg.data)
