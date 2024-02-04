load("/home/hussain/me/projects/nvbts/calib.mat")
focal_length = [intrinsics.camera_matrix(1, 1), intrinsics.camera_matrix(2, 2)];

tangential = intrinsics.dist_coeffs(1:2);

princ = intrinsics.camera_matrix(1:2, 3);

int = cameraIntrinsics(focal_length, princ, [346, 260]);
calib = estworldpose(image_points, world_points, int);
calib.A

cam_pose_wrt_tactile = estworldpose(image_points, world_points, int, "MaxNumTrials", 10000, "MaxReprojectionError", 0.5)
tactile_pose_wrt_cam = invert(cam_pose_wrt_tactile)
tactile_pose_wrt_cam.R;
(rotm2eul(tactile_pose_wrt_cam.R, "XYZ"));
rotm2eul(cam_pose_wrt_tactile.R, "XYZ");

% print the results
fprintf("Tacile pose wrt camera\n");
tactile_pose_wrt_cam.Translation
rotm2eul(tactile_pose_wrt_cam.R, "XYZ")

fprintf("Camera pose wrt tactile\n");
cam_pose_wrt_tactile.Translation
rotm2eul(cam_pose_wrt_tactile.R, "XYZ")