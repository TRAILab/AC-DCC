function [residual_vector_reshape, J_total, L2error] = DCCPoseLoopResidual(parameter_container, measurement_set, dcc_obj)

if dcc_obj.optimize_theta_flag
    optimize_theta_flag_vec = dcc_obj.optimize_theta_flag_vec;
    joint_angles = measurement_set.theta_vec;
    for i=1:length(joint_angles)
        if optimize_theta_flag_vec(i)==1
            if i==1 && dcc_obj.use_modified_DH_flag
                key = strcat('mdh_theta_',int2str(measurement_set.num),'_',int2str(i));
                idx = parameter_container.getKeyIndex(key);
            else
                key = strcat('dh_theta_',int2str(measurement_set.num),'_',int2str(i));
                idx = parameter_container.getKeyIndex(key);
            end
            joint_angles(i) = parameter_container.parameter_list{idx}.parameter.value;
        end
    end
    [T_SM_est, transform_chain] = movingToStaticChain(parameter_container, joint_angles, dcc_obj);
else
    [T_SM_est, transform_chain] = movingToStaticChain(parameter_container, measurement_set.theta_vec, dcc_obj);
end

% The line below gets the transformation from target to dynamic camera,
% through our current estimate of the calibration parameters.
temp_T_SW = measurement_set.T_CW{str2double(dcc_obj.static_cam_key(4))+1};
Tx_MW_pixres = inv(temp_T_SW\T_SM_est);
gimbal_pts = applyTransform(Tx_MW_pixres, measurement_set.target_points{1});
projected_pixels = dcc_obj.cameras{1}.project(gimbal_pts);
L2error = getL2Error(projected_pixels, measurement_set.pixels{1});

% Compute the boxminus Jacobian
T_SM_estimate = Transformation(T_SM_est);
T_SM_measured = Transformation(measurement_set.T_SM{str2double(dcc_obj.static_cam_key(4))});

% boxminus residual
[residual_vector_reshape, ~, J_manifold_minus_right] = T_SM_measured.manifoldMinusAndJacobian(T_SM_estimate);

J_row_block = PoseLoopJacobian(parameter_container, joint_angles, dcc_obj, transform_chain);
J_total = J_manifold_minus_right*J_row_block;

% This is for evaluation
if dcc_obj.eval_flag || dcc_obj.pixel_error_formulation
    T_WM_meas= Transformation(inv(measurement_set.camera_T_target{1}));
    T_WS_meas= Transformation(inv(measurement_set.camera_T_target{str2double(dcc_obj.static_cam_key(4))+1}));
    [T_WM_est, ~, J_right1] = T_WS_meas.composeAndJacobian(T_SM_estimate);
    [residual_vector_reshape, ~, J_right2] = T_WM_meas.manifoldMinusAndJacobian(T_WM_est);
    J_total = J_right2*J_right1*J_row_block;
end