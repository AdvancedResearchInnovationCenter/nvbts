#!/usr/bin/python3


import json
import numpy as np
import cv2
from scipy.optimize import minimize

from objective import Cam2Sensor
from experimental_marker_locations import experimental_marker_array

camera_matrix = np.array([[438.59222618,   0.,         323.89686632],
                          [  0.  ,       439.81991094, 240.11146683],
                          [0., 0., 1.]])
distortion_coeffs = np.array([0.11232303654662454, -0.47277989637474027, -6.726553087628563e-05, 0.0005557680260483787, 0.5754486589853096])

markers_Fs = np.zeros((7,9,3))
x_m = np.linspace(0,8*1.5,9)
y_m = np.linspace(0,6*1.5,7)
z_m = np.zeros((1, 1))
# print(x_m)
# print(y_m)

markers_Fs = np.stack(np.meshgrid(x_m, y_m, z_m))[:, :, :, 0].transpose((1, 2, 0))
markers_exp = experimental_marker_array()

cam2sensor = Cam2Sensor(camera_matrix, markers_exp, markers_Fs)

init_condition = [0, 0, 30, 0, 0., 0] #%x y z rx ry rz

bnds = [(-0.03, 0.03), (-0.03, 0.03), (-0.06, 0.06), (-10, 10), (-10, 10), (-10, 10)]

opt_res = minimize(
    cam2sensor.obj,
    init_condition,
    tol=1e-16
    # bounds=bnds
)

np.set_printoptions(suppress=True)
print(np.array(opt_res.x))

proj_th = cam2sensor.project_markers(opt_res.x, th_markers_arr=markers_Fs)
import matplotlib.pyplot as plt

plt.scatter(*proj_th.T, c='r')#, alpha=np.arange(1, 64) / 128 + 1/2)
plt.scatter(*markers_exp.reshape(63,2).T, c='k')#, alpha=np.arange(1, 64) / 128 + 1/2)
plt.show()
