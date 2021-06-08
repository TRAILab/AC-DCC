function [ J_total_chain,J_total_scale_offset] = movingToStaticChainJacobianReprojection(...
    parameter_container, joint_angles, configuration_object, transform_chain, point)
%CALCULATE_TOTAL_TRANSFORM Calculates the total transformation chain jacobian, using
% the joint_angles and estimation_parameters.  This form does not assume
% a chain type, but instead looks at configuration_object to determine which variables
% should be optimized.

% generate the transform chain manager.
chain_helper  = generate_chain_helper(transform_chain);

% Compute the jacobian using the chain helper.
chain_length = length(transform_chain);
J_total_chain = [];
J_total_scale_offset = [];
for i=1:chain_length
    chain = chain_helper{i};
    point_current = chain.current*[point;1];
    % Jacobian wrt current parameters.
    % If it's the first or last in the chain, we use the 6DOF params.
    if (i==1)
        idx = parameter_container.getKeyIndex('T_EM');
        T_M = parameter_container.parameter_list{idx}.parameter;
        [~,J_current] = T_M.transformAndJacobian(point_current(1:3));
    elseif (i==chain_length)
        idx = parameter_container.getKeyIndex('T_SB');
        T_S = parameter_container.parameter_list{idx}.parameter;
        [~,J_current] = T_S.transformAndJacobian(point_current(1:3));
    else
        % use the DH params.
        
        link_opt = configuration_object.link_struct(end-i+1); % plus one because first link is 6dof transform
        link_theta= joint_angles(end-i+2);
        link_d = link_opt.default_values(2);
        link_a = link_opt.default_values(3);
        link_alpha = link_opt.default_values(4);
        jacobian_cols = [0 0 0];
        
        if(link_opt.index_map(1)>0) %theta
            idx = dh_key_idx('dh_theta_',i-1,parameter_container);
            link_theta = parameter_container.parameter_list{idx}.parameter.value;
        end
        if(link_opt.index_map(2)>0) %d
            idx = dh_key_idx('dh_d_',i-1,parameter_container);
            link_d = parameter_container.parameter_list{idx}.parameter.value;
            jacobian_cols(1) = 1;
        end
        if(link_opt.index_map(3)>0)%a
            idx = dh_key_idx('dh_r_',i-1,parameter_container);
            link_a = parameter_container.parameter_list{idx}.parameter.value;
            jacobian_cols(2) = 1;
        end
        if(link_opt.index_map(4)>0) %alpha
            idx = dh_key_idx('dh_alpha_',i-1,parameter_container);
            link_alpha = parameter_container.parameter_list{idx}.parameter.value;
            jacobian_cols(3) = 1;
        end
        
        % Add the scaling and offsets to the joint angles
        
        if(configuration_object.optimize_scale_offset)
            idx = dh_key_idx('joint_scale_',i-1,parameter_container);
            scaling = parameter_container.parameter_list{idx}.parameter.value;
            
            if(i==2 || i==(chain_length-1)) % only need scale offset
                theta_corrected = link_theta*scaling + link_opt.offset;
                J_scale_offset = [link_theta];
            else % Need to add both scale and offset
                idx = dh_key_idx('joint_offset_',i-1,parameter_container);
                offset = parameter_container.parameter_list{idx}.parameter.value;
                theta_corrected = link_theta*scaling + offset+ link_opt.offset;
                J_scale_offset = [link_theta 1];
            end
        else
            % Add offset to joint angles
            theta_corrected = add_offsets_to_angles(link_theta, link_opt);
        end
        
        % Calculate DH jacobians
        [J_d,~] = dJacobian(theta_corrected, link_d, link_a,link_alpha,point_current(1:3));
        [J_r,~] = rJacobian(theta_corrected, link_d, link_a,link_alpha,point_current(1:3));
        [J_alpha,~] = alphaJacobian(theta_corrected, link_d, link_a,link_alpha,point_current(1:3));
        [J_theta,~] = thetaJacobian(theta_corrected, link_d, link_a,link_alpha,point_current(1:3));
        J_current = [J_d J_r J_alpha];
        % keep only the columns for the parameters we are estimating.
        J_current = J_current(:,find(jacobian_cols));
    end
    
    % Set the final Jacobains using chain rule
    
    % Jacobian wrt point output of current frame.  This works out to be only
    % the rotation component of the chain.
    J_post = chain.post(1:3,1:3);
    J_link = J_post*J_current;
    % Append this to the total Jacobian
    J_total_chain = [J_total_chain J_link];
    if(configuration_object.optimize_scale_offset && i~=1 && i~=chain_length) 
        J_scale_offset_chain = J_post*J_theta*J_scale_offset;
        J_total_scale_offset = [J_total_scale_offset J_scale_offset_chain];
    end
    
end
end















