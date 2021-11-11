function [dcc_obj, opt_problem] = calibrateMechanism(dcc_obj, opt_problem, measurement_set)

%% Description
% This file performs the calibration of the DCC

% Optimize to get calibration parameters
opt_problem.clearLinearSystem();
gradient_norm_threshold = opt_problem.opt_params.gradient_norm_threshold;
step_norm_threshold = opt_problem.opt_params.step_norm_threshold;
max_iterations = opt_problem.opt_params.max_iterations;

num_measurement_sets = length(measurement_set);
for i=1:max_iterations
    avg_pixel_error = zeros(num_measurement_sets, length(dcc_obj.cameras)-1);
    avg_rot_error = zeros(num_measurement_sets, length(dcc_obj.cameras)-1);
    avg_trans_error = zeros(num_measurement_sets, length(dcc_obj.cameras)-1);
    
    % Add all the residual terms.
    for j=1:num_measurement_sets
        measurement_struct = measurement_set{j};
        measurement_struct.num = j;
        measurement_struct.total_num = num_measurement_sets;
        for c=1:length(dcc_obj.cameras)-1
            dcc_obj.static_cam_key = strcat('T_S',num2str(c),'B');
            if ~isempty(measurement_struct.T_SM{c})
                if dcc_obj.reproj_error_formulation
                    calib_error = opt_problem.addDCCReprojectionResidual(measurement_struct, dcc_obj);
                    avg_pixel_error(j,c) = calib_error.pixel_error.mean;
                else
                    calib_error = opt_problem.addDCCPoseLoopResidual(measurement_struct, dcc_obj);
                    avg_pixel_error(j,c) = calib_error.pixel_error.mean;
                    avg_rot_error(j,c) = calib_error.tr_error(1);
                    avg_trans_error(j,c) = calib_error.tr_error(2);
                end
            end
        end
        if length(dcc_obj.cameras)>=3 && dcc_obj.add_identity_residual
            opt_problem.addPoseLoopResidual(measurement_struct, dcc_obj);    
        end
    end
    
    % Plot the error statistics
    avg_error.avg_pixel_error = avg_pixel_error;
    avg_error.avg_rot_error = avg_rot_error;
    avg_error.avg_trans_error = avg_trans_error;
    plotErrorStats(dcc_obj, avg_error);
    if (mod(i,10)==1 || i==max_iterations) && dcc_obj.show_real_world_images
        plotRealWorldImages(dcc_obj, opt_problem, measurement_set, avg_pixel_error);
    end
    
    %% Solve the linear system.
    if dcc_obj.optimize_theta_flag
        opt_problem.J = rearrangeThetaJacobian(dcc_obj, opt_problem.J, measurement_set);
    end
        
    %assert(rank(opt_problem.J)==size(opt_problem.J,2), strcat('The jacobian is rank deficient. Column size: ', ...
    %    num2str(size(opt_problem.J,2)),' but rank is: ',num2str(rank(opt_problem.J))));
    if rank(opt_problem.J)~=size(opt_problem.J,2)
        pause()
    end
    
    [update_delta, success] = opt_problem.solveLinearSystem();

    S=sprintf('Iteration: %d | residual norm: %0.5e | gradient norm: %0.5e | step norm: %0.5e',i, norm(opt_problem.r),norm(opt_problem.g),norm(update_delta));
    disp(S);
    
    % check stopping criteria.
    if( (norm(opt_problem.g)<=gradient_norm_threshold) ||(norm(update_delta)<=step_norm_threshold) || success)
        disp('Reached first order optimality threshold');
        %S=sprintf('Iteration: %d | residual norm: %0.5e | gradient norm: %0.5e | step norm: %0.5e',i, norm(opt_problem.r),norm(opt_problem.g),norm(update_delta));
        %disp(S);
        break;
    end
    
    % Clear system and perform next iteration
    opt_problem.clearLinearSystem();
end

a = 1;