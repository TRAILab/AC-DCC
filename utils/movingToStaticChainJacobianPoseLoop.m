function [J_total_chain, J_theta_MI] = movingToStaticChainJacobianPoseLoop(parameter_container, joint_angles, dcc_obj, transform_chain)

% generate the transform chain manager.
chain_helper = generateChainHelper(transform_chain);

% Compute the jacobian using the chain helper.
chain_length = length(transform_chain);
J_total_chain = [];
J_theta_MI = [];

num_dh_links = dcc_obj.num_DH_links;

for i=1:chain_length
    chain = chain_helper{i};
    % Jacobian wrt current parameters.
    % If it's the first or last in the chain, we use the 6DOF params.
    
    if (i==1)
        if dcc_obj.use_modified_DH_flag
            % use the DH params.
            % phi_pre = chain.pre; % we dont need pre when analyzing
            % dynamic camera joint
            phi_mdh = transform_chain{i};
            phi_post = chain.post;
            
            % Compute the transform and jacobian of the composition from moving
            % camera to current DH block.  Phi_2  = Phi_left*Phi_right =
            % phi_dh*phi_pre;
            PHI_MDH = Transformation(phi_mdh);

            % now compose and jacobian of Phi_chain = Phi_1*Phi_2
            Phi_1 = Transformation(phi_post);
            [~, ~, J_Phi_MDH] = Phi_1.composeAndJacobian(PHI_MDH);
            
            % Now, we finally need the Jacobian wrt to the current DH parameters
            link_opt = dcc_obj.link_struct(i); % plus one because first link is 6dof transform
            dv = link_opt.default_values;
            link_theta = joint_angles(i);
            [link_d, link_a, link_alpha, link_beta, link_y] = deal(dv(2), dv(3), dv(4), dv(5), dv(6));
            jacobian_cols = [0 0 0 0 0];
            
            if(link_opt.index_map(2)>0) %d
                link_d = parameter_container.getKeyValue(strcat('mdh_d_', num2str(i)));
                jacobian_cols(1) = 1;
            end
            if(link_opt.index_map(3)>0)%a
                link_a = parameter_container.getKeyValue(strcat('mdh_r_', num2str(i)));
                jacobian_cols(2) = 1;
            end
            if(link_opt.index_map(4)>0) %alpha
                link_alpha = parameter_container.getKeyValue(strcat('mdh_alpha_', num2str(i)));
                jacobian_cols(3) = 1;
            end
            if(link_opt.index_map(5)>0)%a
                link_beta = parameter_container.getKeyValue(strcat('mdh_beta_', num2str(i)));
                jacobian_cols(4) = 1;
            end
            if(link_opt.index_map(6)>0) %alpha
                link_y = parameter_container.getKeyValue(strcat('mdh_y_', num2str(i)));
                jacobian_cols(5) = 1;
            end
        
            % Add offset to joint angles
            theta_corrected = link_theta + link_opt.offset;

            % Calculate DH jacobians
            MDH_Jacobians = MDHJacobians(theta_corrected, link_d, link_a, link_alpha, link_beta, link_y);
            J_param = MDH_Jacobians(:,2:end);
            J_theta = MDH_Jacobians(:,1);
            
            % keep only the columns for the parameters we are estimating.
            J_param = J_param(:,find(jacobian_cols));

            % now calculate J_current using chain rule
            J_current = J_Phi_MDH * J_param;

            J_theta_total = J_Phi_MDH * J_theta;
            J_theta_MI = [J_theta_MI J_theta_total];
        else
            T_EM = parameter_container.getKeyValue('T_EM');
            T_SE = Transformation(chain.post);
            [~, ~, J_right] = T_SE.composeAndJacobian(T_EM);
            temp = dcc_obj.link_struct(1).index_map;
            J_current = J_right(:,find(temp(:)~=-1));
        end
        
    elseif (i==chain_length)
        if isKey(parameter_container.parameter_key_map, dcc_obj.static_cam_key)
            T_SB = Transformation(parameter_container.getKeyValue(dcc_obj.static_cam_key));
            T_BM = Transformation(chain.pre);
             % composition Jacobian
            [~, J_left, ~] = T_SB.composeAndJacobian(T_BM);
            % This is used to get which columns based on the index map
            static_idx = str2double(dcc_obj.static_cam_key(4));
            temp = dcc_obj.link_struct(1+num_dh_links+static_idx).index_map;
            J_current = J_left(:,find(temp(:)~=-1));
        else
            phi_4dof = transform_chain{i};
            phi_pre = chain.pre;
            
            % Compute the transform and jacobian of the composition from moving
            % camera to current DH block.  Phi_2  = Phi_left*Phi_right =
            % phi_dh*phi_pre;
            PHI_4DOF = Transformation(phi_4dof);

            % now compose and jacobian of Phi_chain = Phi_1*Phi_2
            Phi_1 = Transformation(phi_pre);
            [~, J_Phi_4dof, ~] = PHI_4DOF.composeAndJacobian(Phi_1);
            
            static_idx = str2double(dcc_obj.static_cam_key(4));
            link_opt = dcc_obj.link_struct(1+num_dh_links+static_idx); % plus one because first link is 6dof transform
            dv = link_opt.default_values;
            [rx_val, ry_val, tx_val, ty_val] = deal(dv(1), dv(2), dv(3), dv(4));
            jacobian_cols = [0 0 0 0];
            if(link_opt.index_map(1)>0)
                rx_val = parameter_container.getKeyValue(strcat('4dof_rx_', num2str(static_idx)));
                jacobian_cols(1) = 1;
            end
            if(link_opt.index_map(2)>0)
                ry_val = parameter_container.getKeyValue(strcat('4dof_ry_', num2str(static_idx)));
                jacobian_cols(2) = 1;
            end
            if(link_opt.index_map(3)>0)
                tx_val = parameter_container.getKeyValue(strcat('4dof_tx_', num2str(static_idx)));
                jacobian_cols(3) = 1;
            end
            if(link_opt.index_map(4)>0)
                ty_val = parameter_container.getKeyValue(strcat('4dof_ty_', num2str(static_idx)));
                jacobian_cols(4) = 1;
            end

            J_param = FourDOFJacobians(rx_val, ry_val, tx_val, ty_val);
            % keep only the columns for the parameters we are estimating.
            J_param = J_param(:,find(jacobian_cols));

            % now calculate J_current using chain rule
            J_current = J_Phi_4dof * J_param;
        end
        
    else
        % use the DH params.
        phi_pre = chain.pre;
        phi_dh = transform_chain{i};
        phi_post = chain.post;
       
        % Compute the transform and jacobian of the composition from moving
        % camera to current DH block.  Phi_2  = Phi_left*Phi_right =
        % phi_dh*phi_pre;
        PHI_DH = Transformation(phi_dh);
        PHI_PRE = Transformation(phi_pre);
        
        % compose and jacobian
        [Phi_2, J_Phi_dh, ~] = PHI_DH.composeAndJacobian(PHI_PRE);
        
        % now compose and jacobian of Phi_chain = Phi_1*Phi_2
        Phi_1 = Transformation(phi_post);
        [~, ~, J_Phi_2] = Phi_1.composeAndJacobian(Phi_2);
        
        % Now, we finally need the Jacobian wrt to the current DH
        % parameters
        link_opt = dcc_obj.link_struct(i); % plus one because first link is 6dof transform
        dv = link_opt.default_values;
        if dcc_obj.use_modified_DH_flag
            i = i+1; % We do this because in the modified DH parameter, this would be the second angle
        end
        link_theta = joint_angles(i-1);
        [link_d, link_a, link_alpha] = deal(dv(2), dv(3), dv(4));
        jacobian_cols = [0 0 0];
        
        if(link_opt.index_map(2)>0) %d
            link_d = parameter_container.getKeyValue(strcat('dh_d_', num2str(i-1)));
            jacobian_cols(1) = 1;
        end
        if(link_opt.index_map(3)>0)%a
            link_a = parameter_container.getKeyValue(strcat('dh_r_', num2str(i-1)));
            jacobian_cols(2) = 1;
        end
        if(link_opt.index_map(4)>0) %alpha
            link_alpha = parameter_container.getKeyValue(strcat('dh_alpha_', num2str(i-1)));
            jacobian_cols(3) = 1;
        end
        
        % Add offset to joint angles
        theta_corrected = link_theta + link_opt.offset;
        
        % Calculate DH jacobians
        DH_Jacobians = DHJacobians(theta_corrected, link_d, link_a, link_alpha);
        J_param = DH_Jacobians(:,2:end);
        J_theta = DH_Jacobians(:,1);
        
        % keep only the columns for the parameters we are estimating.
        J_param = J_param(:,find(jacobian_cols));
        
        % now calculate J_current using chain rule
        J_current = J_Phi_2 * J_Phi_dh * J_param;
        
        % update the Jacobians required for the MI NBV formulation
        % (note we still need to apply chain rule!)
        J_theta_total = J_Phi_2 * J_Phi_dh * J_theta;
        J_theta_MI = [J_theta_MI J_theta_total]; 
        
    end
    % Set the final Jacobains using chain rule
    
    % Append this to the total Jacobian
    J_total_chain = [J_total_chain J_current];
end

J_theta_MI = J_theta_MI(:,find(dcc_obj.optimize_theta_flag_vec));
end















