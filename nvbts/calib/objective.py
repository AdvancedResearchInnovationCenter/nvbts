#!/usr/bin python

import numpy as np
from scipy.spatial.transform import Rotation as Rot

class Cam2Sensor:
    
    def __init__(self, camera_matrix, exp_markers_arr, th_markers_arr: np.ndarray):
        self.camera_matrix = camera_matrix
        self.exp = exp_markers_arr
        self.th = th_markers_arr
        self.num_markers = self.exp.shape[0]
        
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]


    def project_markers(self, T, th_markers_arr: np.ndarray):
        R = Rot.from_rotvec(T[-3:]).as_matrix()
        t = np.array(T[:3])
        
        th_Fcam = R @ th_markers_arr.reshape(self.num_markers, 3).T + t.reshape(-1, 1) # 3 x n
        proj_th_x = self.fx * th_Fcam[0, :]/th_Fcam[2,:] + self.cx
        proj_th_y = self.fy * th_Fcam[1, :]/th_Fcam[2,:] + self.cy
        proj_th = np.vstack([proj_th_x, proj_th_y]).T
        
        return proj_th # n x 2
        
    def obj(self, T):
        """Objective function. Will take a 1x6 transformation matrix from the sensor to the camera. 
        It will compute the projection of self.th onto the camera plane and compare it with self.exp

        Args:
            T (np.ndarray): 4x4 transformation matrix
        """
        
        R = Rot.from_euler('xyz', T[-3:]).as_matrix()
        t = np.array(T[:3])
        
        th_Fcam = R @ self.th.reshape(self.num_markers, 3).T + t.reshape(-1, 1) # 3 x n
        
        proj_th_x = self.fx * th_Fcam[0, :]/th_Fcam[2,:] + self.cx
        proj_th_y = self.fy * th_Fcam[1, :]/th_Fcam[2,:] + self.cy
        proj_th = np.vstack([proj_th_x, proj_th_y]).T
        
        err = (proj_th - self.exp.reshape(proj_th.shape))**2
        err = np.mean(err.sum(1))
        # print(th_Fcam - self.exp)
        
        return err
        
        