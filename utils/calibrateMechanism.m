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
    
    % Add all the residual terms.
    for j=1:num_measurement_sets
        j
        measurement_struct = measurement_set{j};
        measurement_struct.num = j;
        measurement_struct.total_num = num_measurement_sets;
        for c=1:num_static_cameras-1
            dcc_obj.static_cam_key = strcat('T_S',num2str(c),'B');
            if ~isempty(measurement_struct.T_SM{c})
                opt_problem.addDCCPoseLoopResidual(measurement_struct, dcc_obj);
            end
        end
        if length(dcc_obj.cameras)>=3 && dcc_obj.add_identity_residual
            opt_problem.addPoseLoopResidual(measurement_struct, dcc_obj);    
        end
    end
    
%     plot_pixel_error_stats(dcc_obj, measurement_avg_pixel_error);
%     if mod(i,10)==1 && dcc_obj.show_real_world_images
%         display_real_world_pixel_error(dcc_obj, opt_problem, measurement_set, measurement_avg_pixel_error);
%     end
    
    %% Solve the linear system.
    if dcc_obj.optimize_theta_flag
        opt_problem.J = rearrangeThetaJacobians(dcc_obj, opt_problem.J, measurement_set);
    end
    
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
    
    % Clear system and perform next iteration
    opt_problem.clearLinearSystem();
end

a = update_delta_full;