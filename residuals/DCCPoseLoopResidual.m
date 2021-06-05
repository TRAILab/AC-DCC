function [residual_whitened, J_whitened_params, J_whitened_thetas] = DCCPoseLoopResidual(parameter_container, measurement_set, dcc_obj)

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

T_SW_meas_mat = measurement_set.T_CW{str2double(dcc_obj.static_cam_key(4)) + 1};
T_SW_meas = Transformation(T_SW_meas_mat);

T_MW_meas = Transformation(measurement_set.T_CW{1});

[T_SW_est, J_SM_est, J_MW_meas] = T_SM_est.composeAndJacobian(T_MW_meas);

% boxminus residual
[residual_vector, J_mm_left, J_mm_right] = T_SW_meas.manifoldMinusAndJacobian(T_SW_est);

T_SW_cov = measurement_set.T_CW_cov{str2double(dcc_obj.static_cam_key(4)) + 1};
T_MW_cov = measurement_set.T_CW_cov{1};

J2 = J_mm_right*J_MW_meas;

comb_cov = J_mm_left*T_SW_cov*J_mm_left' + J2*T_MW_cov*J2';
comb_info = inv(comb_cov);
L = chol(comb_info,'lower');

[J_params, J_thetas] = TSMJacobian(parameter_container, joint_angles, dcc_obj, transform_chain);

J_total_params = J_mm_right*J_SM_est*J_params;
J_total_thetas = J_mm_right*J_SM_est*J_thetas;

J_whitened_params = L'*J_total_params;
J_whitened_thetas = L'*J_total_thetas;
residual_whitened = L'*residual_vector;