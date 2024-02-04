import json
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

class Cam2SensorCalib:

    def __init__(self, intrinsincs_path):
        """Class to do camera to sensor calibration

        Args:
            intrinsincs_path (str): calibration file path. from ARIC's calibration tool https://github.com/AdvancedResearchInnovationCenter/aric-camera-calibration
        """
        self._3d_markers_arr = None
        self._2d_markers_arr = None

        with open(intrinsincs_path, 'r') as f:
            self._intrinsics = json.load(f)


    def set_3d_markers(self, markers_arr):
        """Set the 3D markers. See utils for dome shaped markers and flat markers.

        Args:
            markers_arr (np.ndarray):  n x 3 3D markers array
        """
        self._3d_markers_arr = markers_arr

    def set_2d_markers(self, markers_arr):
        """Set the 2D markers. Use blob detection or any other method to get the 2D markers. Assumes markers are sorted.

        Args:
            markers_arr (np.ndarray):  n x 2 2D markers array
        """
        self._2d_markers_arr = markers_arr

    def save_mat(self):
        out = {
            "intrinsics": {
                "camera_matrix": self._intrinsics['camera_matrix'],
                "dist_coeffs": self._intrinsics['distortion_coeffs']
            },
            "image_points": self._2d_markers_arr.tolist(),
            "world_points": self._3d_markers_arr.tolist(),
            }
        
        from scipy.io import savemat
        savemat('calib.mat', out)

        print("Calibration data saved to calib.mat. Run the matlab script to get the calibration data")
        

    def calibrate(self, method = 'pnp'):
        assert self._3d_markers_arr is not None, "3D markers not set"
        assert self._2d_markers_arr is not None, "2D markers not set"

        if method == 'pnp':
            return self._calibrate_pnp()
        elif method == 'matlab':
            self.save_mat()
        else:
            raise ValueError("Invalid method")
        
    def _calibrate_pnp(self):
        # Get the camera intrinsics
        mtx = np.array(self._intrinsics['camera_matrix'])
        dist = np.array(self._intrinsics['distortion_coeffs'])

        # Get the 3D and 2D markers
        objp = self._3d_markers_arr
        imgp = self._2d_markers_arr

        # Get the rotation and translation vectors
        ret, rvecs, tvecs = cv2.solvePnP(objp, imgp, mtx, dist)

        self.reprojection_error = 0

        # Reproject the 3D points to 2D
        imgpts, jac = cv2.projectPoints(objp, rvecs, tvecs, mtx, dist)
        imgpts = imgpts.squeeze()

        # Calculate the reprojection error
        self.reprojection_error = np.mean(np.linalg.norm(imgp - imgpts, axis=1))

        plt.figure()
        plt.plot(imgp[:,0], imgp[:,1], 'ro')
        plt.plot(imgpts[:,0], imgpts[:,1], 'bo')
        plt.text(0, 0, f"Reprojection error: {self.reprojection_error}")
        # plt.show()
        # invert transform
        
        # Get the rotation matrix
        rmat = R.from_rotvec(rvecs.flatten()).as_matrix()
        rmat = rmat.T

        # Get the translation vector
        tvec = -np.dot(rmat, tvecs.flatten())

        self.rmat = rmat
        self.tvec = tvec

        print(f"Reprojection error: {self.reprojection_error}")
        print(f"Euler angles: {R.from_matrix(rmat).as_euler('xyz')}")
        print(f"Translation: {tvec}")

        return R.from_matrix(rmat).as_euler('xyz'), tvec
    

