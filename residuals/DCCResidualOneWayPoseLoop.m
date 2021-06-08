function [residual, J_total, calib_error] = DCCResidualOneWayPoseLoop(parameter_container, measurement_set, dcc_obj)

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

T_SW_meas_mat = measurement_set.T_CW{static_cam_num + 1};
T_SW_meas = Transformation(T_SW_meas_mat);
T_MW_meas = Transformation(measurement_set.T_CW{1});

[T_SW_est, J_SM_est, ~] = T_SM_est.composeAndJacobian(T_MW_meas);
[residual, ~, J_mm_right] = T_SW_meas.manifoldMinusAndJacobian(T_SW_est);

J_total = J_mm_right*J_SM_est*[J_chain J_theta];

static_pts = applyTransform(T_SW_est.matrix, measurement_set.target_points{static_cam_num+1});
projected_pix = dcc_obj.cameras{static_cam_num + 1}.project(static_pts);
calib_error.pixel_error = getL2Error(projected_pix, measurement_set.pixels{static_cam_num+1});

abs_residual = abs(residual);
calib_error.tr_error = [rad2deg(mean(abs_residual(1:3))) mean(abs_residual(4:6))];