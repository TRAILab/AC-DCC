function [avg_err, cam_T_target] = pnp_sanity_check(target_pts, true_pixels, camera, reprojection_threshold, cam_T_target_init)

% This function is used to test if we get a better PnP than the one
% provided by Kalibr

[cam_T_target, ~, ~] = solve_pnp_bundle_adjustment(target_pts, true_pixels, camera, reprojection_threshold, cam_T_target_init);

target_pts_in_camera = applyTransform(cam_T_target, target_pts);
projected_pixels = general_camera_projection(camera.intrinsics, target_pts_in_camera);

% compute the errors
error_matrix = projected_pixels - true_pixels;
error_vector = sqrt(sum(error_matrix.^2,2));
min_err = min(error_vector);
max_err = max(error_vector);
avg_err = mean(error_vector);
%min_err_rig(j) = min_err;
%max_err_rig(j) = max_err;
%avg_err_rig(j) = avg_err;

