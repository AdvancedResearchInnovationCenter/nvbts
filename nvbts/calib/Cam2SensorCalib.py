import json
import numpy as np

class Cam2SensorCalib:

    def __init__(self):
        """Class to do camera to sensor calibration

        Args:
            intrinsincs_path (str): calibration file path. from ARIC's calibration tool https://github.com/AdvancedResearchInnovationCenter/aric-camera-calibration
        """
        self._3d_markers_arr = None
        self._2d_markers_arr = None


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
        
