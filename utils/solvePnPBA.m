function [cam_T_target, success, L2error] = solvePnPBA(target_pts_3D, pixels, camera_object, reprojection_threshold, T_CW_init)

% Function returns the transformation from target frame to camera frame.
% This version uses bundle adjustment, not homography, so it can work
% without the planar assumption.

% set up the initial guess
T_CW = Transformation([0 0 0 0 0 0]);
T_CW.matrix = T_CW_init;

% only going to optimize over one parameter
O1 = OptimizationParameter(T_CW,6);

% set up the parameter container
parameter_container = ParameterContainer;
parameter_container.addParameter(O1);

% set up the optimization problem
opt_problem = Problem(parameter_container);

% perform the optimization
gradient_norm_threshold = 1e-6;
step_norm_threshold = 1e-10;
max_iterations = 500;
success = 0;
num_points = length(target_pts_3D);

for i=1:max_iterations
    
    % Add all the residual terms.
    for j=1:num_points
        measurement_struct = {target_pts_3D(j,:) pixels(j,:)};
        opt_problem.addUnaryReprojectionResidual(1, measurement_struct, camera_object);
    end
    
    % solve the linear system.
    update_delta = opt_problem.solveLinearSystem();
    opt_problem.updateParameters(0.25*update_delta, []);
    S = sprintf('Iteration: %d | residual norm: %0.5e | gradient norm: %0.5e | step norm: %0.5e',i, norm(opt_problem.r),norm(opt_problem.g),norm(update_delta));
    %disp(S);
    
    % check stopping criteria.
    if( (norm(opt_problem.g)<=gradient_norm_threshold) ||(norm(update_delta)<=step_norm_threshold))
        disp('Reached first order optimality threshold');
        S=sprintf('Iteration: %d | residual norm: %0.5e | gradient norm: %0.5e | step norm: %0.5e',i, norm(opt_problem.r),norm(opt_problem.g),norm(update_delta));
        disp(S);
        success = 1;
        break;
    end
    % Clear system and perform next iteration
    opt_problem.clearLinearSystem();
end

cam_T_target.matrix = T_CW.matrix;
cam_T_target.cov = opt_problem.getSystemCovariance();

target_pts_in_cam_frame = applyTransform(cam_T_target.matrix, target_pts_3D);
projected_pixels = camera_object.project(target_pts_in_cam_frame);
L2error = getL2Error(projected_pixels, pixels);

if(L2error.mean < reprojection_threshold)
    success = 1;
end

end