function [residual, J_total, calib_error] = DCCReprojectionResidual(parameter_container, measurement_set, dcc_obj)

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

T_MW_meas = Transformation(measurement_set.T_CW{1});
[T_SW_est, J_SM, ~] = T_SM_est.composeAndJacobian(T_MW_meas);

common_target_points = measurement_set.common_target_points{static_cam_num};

proj_pix_list = zeros(size(common_target_points,1), 2);
J_total = zeros(size(common_target_points,1), size(J_chain,2)+size(J_theta,2));

for p=1:size(common_target_points, 1)
    common_target_point = common_target_points(p,:);
    [pt_static, J_T_SW_est, ~] = T_SW_est.transformAndJacobian(common_target_point);
    [proj_pix, J_proj] = dcc_obj.cameras{static_cam_num + 1}.projectAndJacobian(pt_static);
    proj_pix_list(p,:) = proj_pix;
    J_total(2*p-1:2*p,:) = -1 * J_proj * J_T_SW_est * J_SM * [J_chain J_theta];
end
residual = measurement_set.common_pixels{static_cam_num}-proj_pix_list;
residual = reshape(residual',size(residual,1)*size(residual,2),1);

calib_error.pixel_error = getL2Error(measurement_set.common_pixels{static_cam_num}, proj_pix_list);