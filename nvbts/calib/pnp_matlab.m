load("/home/hussain/me/projects/nvbts/calib.mat");
focal_length = [intrinsics.camera_matrix(1, 1), intrinsics.camera_matrix(2, 2)];

tangential = intrinsics.dist_coeffs(1:2);

princ = intrinsics.camera_matrix(1:2, 3);

int = cameraIntrinsics(focal_length, princ, [346, 260]);
cam_pose_wrt_tactile = estworldpose(image_points, world_points, int);

% cam_pose_wrt_tactile = estworldpose(image_points, world_points, int, "MaxNumTrials", 100000, "MaxReprojectionError", 0.5)
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

imagePointsProjected = world2img(world_points, tactile_pose_wrt_cam, int);

% Plot image points
figure;
hold on;
plot(image_points(:, 1), image_points(:, 2), 'r.', 'MarkerSize', 20);
plot(imagePointsProjected(:, 1), imagePointsProjected(:, 2), 'b+', 'MarkerSize', 10);
legend('Image Points', 'Projected World Points');
title('Image Points and Projected World Points');