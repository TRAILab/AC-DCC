classdef Problem<handle
    %PROBLEM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Full parameter container
        parameter_container;
        % system jacobian
        J = [];
        % residual vector
        r = [];
        % gradient vector;
        g = [];
    end
    
    methods
        function obj =Problem(parameter_container)
            obj.parameter_container = parameter_container;
        end
        
        function [] =  addResidual(obj, type,key_list,measurement_struct,camera)
            
            if(strcmp(type,'UnaryReprojection'))
                % since it's a unary factor, should just have 1 key
                key = key_list(1);
                
                point_meas = measurement_struct{1};
                pixel_meas = measurement_struct{2};
                
                parameter_object = obj.parameter_container.parameter_list{key};
                parameter_value = parameter_object.parameter;
                [residual, J_T] = reprojectionErrorResidual(camera, parameter_value,point_meas,pixel_meas);
                
                %only add this residual if valid
                if(~isempty(residual))
                    meas_size = length(residual);
                    % append the residual vector
                    obj.r = [obj.r; residual];
                    % append the jacobian matrix
                    row_index = size(obj.J,1)+1;
                    column_index = parameter_object.column_idx;
                    column_size = parameter_object.tangentspace_dim;
                    obj.J(row_index:row_index+meas_size-1,column_index:column_index+column_size-1) = J_T;
                end   
            else
                error('Unknown residual type')
            end
        end
        
        function [] = addPoseLoopResidual(obj, measurement_struct, configuration_object)
            meas_s1_T_s2 = Transformation(zeros(1,6));
            % s1_T_w * inv(s2_T_w);
            meas_s1_T_s2.matrix = measurement_struct.camera_T_target{2}/measurement_struct.camera_T_target{3}; % T_S1S2
            idx_S1B = obj.parameter_container.getKeyIndex('T_S1B');
            idx_S2B = obj.parameter_container.getKeyIndex('T_S2B');
            opt_s1_T_b = obj.parameter_container.parameter_list{idx_S1B}.parameter;
            opt_s2_T_b = obj.parameter_container.parameter_list{idx_S2B}.parameter;
            %opt_s1_T_s2.matrix = opt_s1_T_b.matrix/opt_s2_T_b.matrix;
            %[residual_vector_reshape, ~, J_manifold_minus_right] = meas_s1_T_s2.manifoldMinusAndJacobian(opt_s1_T_s2);
            [opt_b_T_s2, J_inv] = opt_s2_T_b.inverseAndJacobian();
            [opt_s1_T_s2, J_s1, J_s2_inv] = opt_s1_T_b.composeAndJacobian(opt_b_T_s2);
            [residual_vec, ~, J_manifold_right] = meas_s1_T_s2.manifoldMinusAndJacobian(opt_s1_T_s2);

            J_total = [J_manifold_right*J_s1 J_manifold_right*J_s2_inv*J_inv];
            J_total = rearrange_identity_residual_jacobian(configuration_object, J_total);

            %T_S1_S2_cov = computeRelativeStaticPoseCovariance(configuration_object, measurement_struct);
            %info_matrix = inv(T_S1_S2_cov);
            % Perform whitening
            %[r_whiten, J_whiten] = prewhiten_residual(residual_vec, J_total, info_matrix);
            % Need to do whitening
            obj.r = [obj.r; residual_vec];
            obj.J = [obj.J; J_total];
        end
        
        
        function [reprojection_vector] = addDCCResidualReprojection(obj, measurement_struct,configuration_object)
            
            [residual_vector_whiten, J_whiten,reprojection_vector] = DCCResidualOneWayReprojection(obj.parameter_container,measurement_struct,configuration_object);
            
            % append the residual vector
            obj.r = [obj.r; residual_vector_whiten];
            % append the jacobian matrix
            
            obj.J  = [obj.J;J_whiten];
            
        end
        
        function [avg_pixel_error, det_info_mat] = addDCCResidualPoseLoop(obj, measurement_struct, simulation_object)
            [residual_vector, J_total, avg_pixel_error, ~] = DCCResidualOneWayPoseLoop(obj.parameter_container, measurement_struct, simulation_object);
            
            % Rearrange Jacobian. Because it doesnt account for which
            % static camera is being considered
            J_total = rearrange_non_theta_jacobian(simulation_object, obj.parameter_container, J_total);
            
            % compute the relative pose covariance
            cov_mat = computeRelativePoseCovariance(simulation_object, measurement_struct);
            info_mat = inv(cov_mat);
            det_info_mat = det(info_mat);
            
            % Perform whitening
            %[r_whiten, J_whiten] = prewhiten_residual(residual_vector, J_total, info_mat);
            %assert(rank(J_whiten) == size(J_whiten,2))
            
            % append the residual vector
            %obj.r = [obj.r; r_whiten];
            obj.r = [obj.r; residual_vector];
            
            % append the jacobian matrix
            %obj.J = [obj.J; J_whiten];
            obj.J = [obj.J; J_total];
        end
        
        function [] = addDCCCovariancePoseloopResidual(obj)
            
            % residual of the form: I - inv(T_S_WS.matrix)*T_S_M.matrix*T_M_WM.matrix
            T_I = Transformation([0 0 0 0 0 0]);
            p_M_WM = obj.parameter_container.parameter_list{1};
            p_S_WS = obj.parameter_container.parameter_list{2};
            p_S_M = obj.parameter_container.parameter_list{3};
            
            T_S_WS = p_S_WS.parameter;
            T_M_WM = p_M_WM.parameter;
            T_S_M = p_S_M.parameter;
            
            % Jacobian calculations
            [T_S_MW, J_left, J_right] = T_S_M.composeAndJacobian(T_M_WM);
            [T_WS_S, J_inv] = T_S_WS.inverseAndJacobian();
            [T_WS_MW, J_left2, J_right2] = T_WS_S.composeAndJacobian(T_S_MW);
            
            [residual, ~, Jmm] = T_I.manifoldMinusAndJacobian(T_WS_MW);
            meas_size = length(residual);
            
            % Jacobian wrt param1
            J1 = Jmm*J_left2*J_inv;
            
            % Jacobian wrt param2
            J2 = Jmm*J_right2*J_right;
            
            % Jacobian wrt param3
            J3 = Jmm*J_right2*J_left;
            
            % append the residual vector
            obj.r = [obj.r; residual];
            row_index = size(obj.J,1)+1;
            
            % Append the Jacobians
            % J1
            column_index = p_S_WS.column_idx;
            column_size = p_S_WS.tangentspace_dim;
            obj.J(row_index:row_index+meas_size-1,column_index:column_index+column_size-1) = J1;
            
            % J2
            column_index = p_M_WM.column_idx;
            column_size = p_M_WM.tangentspace_dim;
            obj.J(row_index:row_index+meas_size-1,column_index:column_index+column_size-1) = J2;
            
            % J3
            column_index = p_S_M.column_idx;
            column_size = p_S_M.tangentspace_dim;
            obj.J(row_index:row_index+meas_size-1,column_index:column_index+column_size-1) = J3; 
        end
         
        function update_delta = solveLinearSystem(obj)
            H = obj.J'*obj.J;
            obj.g = -obj.J'*obj.r;
            %if isKey(obj.parameter_container.parameter_key_map,'dh_theta_1_1') || isKey(obj.parameter_container.parameter_key_map,'dh_theta_1_2') || isKey(obj.parameter_container.parameter_key_map,'dh_theta_1_3')
               H_sparse = sparse(H);
               L = chol(H_sparse,'lower');
               update_delta = (L'\(L\obj.g));
               %L = chol(H_sparse,'lower');
               %L = chol(H,'lower');
               %update_delta = (L'\(L\obj.g));
            %else
                %opts.SYM = true;
                %opts.POSDEF = true;
                %H = H + 0.0001*eye(size(H));
                %update_delta = linsolve(H, obj.g, opts);
            %end
        end
        
        function [] = updateParameters(obj, system_update_delta, simulation_object)
            
            if isempty(simulation_object)
                num_parameters = length(obj.parameter_container.parameter_list);
                for i=1:num_parameters
                    param_block_size = obj.parameter_container.parameter_list{i}.tangentspace_dim;
                    param_column_idx = obj.parameter_container.parameter_list{i}.column_idx;
                    param_update_vector = system_update_delta(param_column_idx:param_column_idx+param_block_size-1);
                    obj.parameter_container.parameter_list{i}.parameter.manifoldPlus(param_update_vector);
                end
            else
                own_update_params(obj, system_update_delta, simulation_object)
            end
            
        end
        
        function cov = getSystemCovariance(obj)
            H = obj.J'*obj.J;
            cov = inv(H);
        end
        
        function [] = clearLinearSystem(obj)
            obj.r = [];
            obj.J = [];
            obj.g = [];
        end
        
    end
    
end

