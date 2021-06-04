function [dcc_obj, opt_problem] = calibrateMechanism(dcc_obj, opt_problem, opt_params, measurement_set)

update_delta_full = [];
pause_flag = 0;

% Optimize to get calibration parameters
opt_problem.clearLinearSystem();
gradient_norm_threshold = opt_params.gradient_norm_threshold;
step_norm_threshold = opt_params.step_norm_threshold;
max_iterations = opt_params.max_iterations;

num_measurement_sets = length(measurement_set);
num_static_cameras = length(dcc_obj.cameras);

for i=1:max_iterations
    measurement_avg_pixel_error = zeros(num_static_cameras, num_measurement_sets);
    measurement_det_info_mat = zeros(num_static_cameras, num_measurement_sets);
    
    % Add all the residual terms.
    for j=1:num_measurement_sets
        measurement_struct = measurement_set{j};
        measurement_struct.num = j;
        measurement_struct.total_num = num_measurement_sets;
        for c=1:num_static_cameras
            dcc_obj.static_cam_key = strcat('T_S',num2str(c),'B');
            if ~isempty(measurement_struct.T_SM{c})
                [avg_pixel_error, ~] = opt_problem.addDCCPoseLoopResidual(measurement_struct, dcc_obj);
                measurement_avg_pixel_error(c,j) = avg_pixel_error;
            end
        end
        if length(dcc_obj.cameras)>=3 && dcc_obj.add_identity_residual
            opt_problem.addPoseLoopResidual(measurement_struct, dcc_obj);    
        end
    end
    
    plot_pixel_error_stats(dcc_obj, measurement_avg_pixel_error);
    if mod(i,10)==1 && dcc_obj.show_real_world_images
        display_real_world_pixel_error(dcc_obj, opt_problem, measurement_set, measurement_avg_pixel_error);
    end
    
    %% Solve the linear system.
    if dcc_obj.optimize_theta_flag
        % Need to arrange the theta values differently
        num_thetas_to_optimize = sum(dcc_obj.optimize_theta_flag_vec);
        Jac_diag_thetas = zeros(size(opt_problem.J,1), num_thetas_to_optimize*num_measurement_sets);
        num_non_theta = size(opt_problem.J,2)-num_thetas_to_optimize;
        Jac_params = opt_problem.J(:,1:num_non_theta);
        Jac_thetas = opt_problem.J(:,num_non_theta+1:end);
        for m=1:num_measurement_sets
            measurement_struct = measurement_set{m};
            empty_counter = 0;
            for c=1:length(measurement_struct.T_S_M)
                if isempty(measurement_struct.T_S_M{c})
                    empty_counter = empty_counter + 1;
                end
            end
            magic_num = length(dcc_obj.camera)-1-empty_counter+dcc_obj.add_identity_residual;
            temp = Jac_thetas(6*magic_num*(m-1)+1:6*magic_num*(m-1)+6*magic_num,:);
            Jac_diag_thetas(6*magic_num*(m-1)+1:6*magic_num*(m-1)+6*magic_num, num_thetas_to_optimize*(m-1)+1:num_thetas_to_optimize*(m-1)+num_thetas_to_optimize) = temp;
        end
        opt_problem.J = [Jac_params Jac_diag_thetas];
        
        % Plot the difference between the true and optimized values
        if dcc_obj.have_true_encoder_values
            figure(4);
            clf;
            optimized_theta_vals = zeros(num_measurement_sets, dcc_obj.num_DH_links);
            optimized_theta_idx = find(dcc_obj.optimize_theta_flag_vec);
            for m=1:num_measurement_sets
                for t=1:length(dcc_obj.optimize_theta_flag_vec)
                    if dcc_obj.optimize_theta_flag_vec(t)
                        theta_name = strcat('dh_theta_',num2str(m),'_',num2str(t));
                        key_idx = opt_problem.parameter_container.parameter_key_map(theta_name);
                        theta_opt_val = opt_problem.parameter_container.parameter_list{key_idx}.parameter.value;
                        optimized_theta_vals(m,t) = theta_opt_val;
                        assert(strcmp(theta_name,opt_problem.parameter_container.parameter_list{key_idx}.key)==1);
                    end
                end
            end
            theta_idx_cols = find(dcc_obj.optimize_theta_flag_vec);
            encoder_angle_diff = rad2deg(abs(dcc_obj.true_encoder_angles(1:num_measurement_sets,theta_idx_cols) - optimized_theta_vals(:,theta_idx_cols)));
            for b=1:size(encoder_angle_diff,2)
                subplot(size(encoder_angle_diff,2),1,b);
                bar(encoder_angle_diff(:,b));
                title(strcat('Angle difference between true and optimized values, angle:',num2str(optimized_theta_idx(b))));
            end
        end
        
        if dcc_obj.show_residual_values && dcc_obj.add_prior_joint_angles
            figure(5);
            clf;
            num_non_theta_values = length(opt_problem.r) - length(theta_error_vec);
            mag_non_theta_residuals = mean(abs(opt_problem.r(1:num_non_theta_values,1)));
            mag_theta_residuals = mean(abs(opt_problem.r(num_non_theta_values+1:end,1)));
            bar([mag_non_theta_residuals mag_theta_residuals]);
            title('Magnitude of residuals');
            legend('Non-theta ; theta');
        end
        
    end
   
    J = opt_problem.J;
    
    if pause_flag
        pause();
        pause_flag = 0;
    end
    
    %[linearly_indep_col_set, remove_cols] = determine_linealy_indep_cols(opt_problem.J);
    %validateDegeneracies(simulation_object, opt_problem, measurement_set);
    assert(rank(opt_problem.J)==size(opt_problem.J,2), strcat('The jacobian is rank deficient. Column size: ', ...
        num2str(size(opt_problem.J,2)),' but rank is: ',num2str(rank(opt_problem.J))));
    
    update_delta = opt_problem.solveLinearSystem();
    if i>100
        opt_problem.updateParameters(0.02*update_delta, dcc_obj);
    else
        opt_problem.updateParameters(0.1*update_delta, dcc_obj);
    end
    %opt_problem.updateParameters(0.2*update_delta, []);
    S=sprintf('Iteration: %d | residual norm: %0.5e | gradient norm: %0.5e | step norm: %0.5e',i, norm(opt_problem.r),norm(opt_problem.g),norm(update_delta));
    disp(S);
    
    update_delta_full = [update_delta_full update_delta];
    
    % check stopping criteria.
    if( (norm(opt_problem.g)<=gradient_norm_threshold) ||(norm(update_delta)<=step_norm_threshold))
        disp('Reached first order optimality threshold');
        %S=sprintf('Iteration: %d | residual norm: %0.5e | gradient norm: %0.5e | step norm: %0.5e',i, norm(opt_problem.r),norm(opt_problem.g),norm(update_delta));
        %disp(S);
        success = 1;
        break;
    end
    
    % Show the simulation Object
    %figure(5);
    %clf;
    [~] = display_simulation_object(dcc_obj, [0 0 0], opt_problem);
    
    % Clear system and perform next iteration
    opt_problem.clearLinearSystem();
end

a = update_delta_full;