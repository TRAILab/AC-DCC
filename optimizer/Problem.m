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
        
        function [] = addUnaryReprojectionResidual(obj, key_list, measurement_struct, camera)
            
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
        
        function [] = addDCCPoseLoopResidual(obj, measurement_struct, dcc_obj)
            [residual_whitened, J_params, J_thetas] = DCCPoseLoopResidual(obj.parameter_container, measurement_struct, dcc_obj);
            
            % Rearrange Jacobian. Because it doesnt account for which static camera is being considered
            J_rearranged = rearrangeNonThetaJacobians(dcc_obj, J_params);

            obj.r = [obj.r; residual_whitened];
            obj.J = [obj.J; [J_rearranged J_thetas]];
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

