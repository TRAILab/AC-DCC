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
        
        function [reprojection_vector] = addDCCResidualReprojection(obj, measurement_struct,configuration_object)
            
            [residual_vector_whiten, J_whiten,reprojection_vector] = DCCResidualOneWayReprojection(obj.parameter_container,measurement_struct,configuration_object);
            
            % append the residual vector
            obj.r = [obj.r; residual_vector_whiten];
            % append the jacobian matrix
            
            obj.J  = [obj.J;J_whiten];
            
        end
        
        function [calib_error] = addDCCResidualPoseLoop(obj, measurement_struct, simulation_object)
            [residual_vector, J_total, calib_error] = DCCResidualOneWayPoseLoop(obj.parameter_container, measurement_struct, simulation_object);
            
            % Rearrange Jacobian. Because it doesnt account for which
            % static camera is being considered
            J_total = rearrangeNonThetaJacobian(simulation_object, J_total);
            
            % append the residual vector
            obj.r = [obj.r; residual_vector];
            
            % append the jacobian matrix
            obj.J = [obj.J; J_total];
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
        
        function [] = updateParameters(obj, system_update_delta)
            num_parameters = length(obj.parameter_container.parameter_list);
            for i=1:num_parameters
                param_block_size = obj.parameter_container.parameter_list{i}.tangentspace_dim;
                param_column_idx = obj.parameter_container.parameter_list{i}.column_idx;
                param_update_vector = system_update_delta(param_column_idx:param_column_idx+param_block_size-1);
                obj.parameter_container.parameter_list{i}.parameter.manifoldPlus(param_update_vector);
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

