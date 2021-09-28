function [white_residual, white_J_total, calib_error] = DCCPoseLoopResidual(parameter_container, measurement_set, dcc_obj)

%% Description
% This file returns the poseloop residual and jacobian.

optimize_theta_flag_vec = dcc_obj.optimize_theta_flag_vec;
joint_angles = measurement_set.theta_vec;
for i=1:length(joint_angles)
    if optimize_theta_flag_vec(i)==1
        if i==1 && dcc_obj.use_modified_DH_flag
            key = strcat('mdh_theta_',int2str(measurement_set.num),'_',int2str(i));
        else
            key = strcat('dh_theta_',int2str(measurement_set.num),'_',int2str(i));
        end
        joint_angles(i) = parameter_container.getKeyValue(key);
    end
end
[T_SM_est_mat, transform_chain] = movingToStaticChain(parameter_container, joint_angles, dcc_obj);
T_SM_est = Transformation(T_SM_est_mat);

% calculate Jacobian
[J_chain, J_theta] = movingToStaticChainJacobianPoseLoop(parameter_container, joint_angles, dcc_obj, transform_chain);

static_cam_num = str2double(dcc_obj.static_cam_key(4));

% Get data for calculating the jacobians
T_SW_meas_mat = measurement_set.T_CW{static_cam_num + 1};
T_SW_meas = Transformation(T_SW_meas_mat);
T_MW_meas = Transformation(measurement_set.T_CW{1});

% Calculate jacobians
[T_SW_est, J_SM_est, J_SW_MW] = T_SM_est.composeAndJacobian(T_MW_meas);
[residual, J_mm_left, J_mm_right] = T_SW_meas.manifoldMinusAndJacobian(T_SW_est);
J_total = J_mm_right*J_SM_est*[J_chain J_theta];

% Calculate covariance and whiten jacobian and residual
static_cov = J_mm_left*measurement_set.T_CW_cov{static_cam_num + 1}*(J_mm_left');
gimbal_cov = (J_mm_right*J_SW_MW)*measurement_set.T_CW_cov{1}*(J_SW_MW'*J_mm_right');
total_cov = static_cov + gimbal_cov;
total_info = inv(total_cov);
L = chol(total_info, 'lower');
white_J_total = L'*J_total;
white_residual = L'*residual;

% Caluclate pixel error and pose error
static_pts = applyTransform(T_SW_est.matrix, measurement_set.target_points{static_cam_num+1});
projected_pix = dcc_obj.cameras{static_cam_num + 1}.project(static_pts);
calib_error.pixel_error = getL2Error(projected_pix, measurement_set.pixels{static_cam_num+1});
abs_residual = abs(residual);
calib_error.tr_error = [rad2deg(mean(abs_residual(1:3))) mean(abs_residual(4:6))];