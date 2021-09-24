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
        % optimization parameters
        opt_params = [];
        
        % Parameters for LM
        LM_init_flag = 0;
        r0 = [];
        L0 = [];
        x0 = {}; % This holds the old state before LM is accepted
        J0 = [];
        lambda = 0; % This is mu from the paper
        tau = 1e-6;
        eps1 = 1e-12;
        eps2 = 1e-12;
        nu = 2;
    end
    
    methods
        function obj = Problem(parameter_container)
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
        
        function [calib_error] = addDCCReprojectionResidual(obj, meas_struct, dcc_obj)
            
            [residual_vector, J_total, calib_error] = DCCReprojectionResidual(obj.parameter_container, meas_struct, dcc_obj);
            
            % Rearrange Jacobian. Because it doesnt account for which
            % static camera is being considered
            J_total = rearrangeNonThetaJacobian(dcc_obj, J_total);
            
            % append the residual vector
            obj.r = [obj.r; residual_vector];
            
            % append the jacobian matrix
            obj.J = [obj.J; J_total];
            
        end
        
        function [calib_error] = addDCCPoseLoopResidual(obj, meas_struct, dcc_obj)
            [residual_vector, J_total, calib_error] = DCCPoseLoopResidual(obj.parameter_container, meas_struct, dcc_obj);
            
            % Rearrange Jacobian. Because it doesnt account for which
            % static camera is being considered
            J_total = rearrangeNonThetaJacobian(dcc_obj, J_total);
            
            % append the residual vector
            obj.r = [obj.r; residual_vector];
            
            % append the jacobian matrix
            obj.J = [obj.J; J_total];
        end
         
        function [update_delta, success] = solveLinearSystem(obj)
            if strcmp(obj.opt_params.opt_type, 'GN')
                H = obj.J'*obj.J;
                obj.g = -obj.J'*obj.r;
                H_sparse = sparse(H);
                L = chol(H_sparse,'lower');
                update_delta = (L'\(L\obj.g));
                obj.updateParameters(0.05*update_delta);
                success = false;
                
            elseif strcmp(obj.opt_params.opt_type, 'LM1')
                 
                % Reset the success flag
                success = false;
                
                % First iteration
                if ~obj.LM_init_flag
                    
                    % Create hessian and gradient
                    A = obj.J'*obj.J;
                    obj.g = obj.J'*obj.r; % Note no negative sign here
                    
                    % initialize lambda
                    obj.lambda = obj.tau*max(diag(A));
                    
                    % Solve the system
                    A = A + obj.lambda*eye(size(A));
                    A_sparse = sparse(A);
                    L = chol(A_sparse,'lower');
                    h_lm = (L'\(L\-obj.g));
                    update_delta = h_lm;
                    
                    % Check for success based on update vector
                    if norm(h_lm) <= obj.eps2*(norm(obj.getState()) + obj.eps2)
                        success = true;
                    else
                        obj.LM_init_flag = 1;
                        obj.L0 = 0.5*h_lm'*(obj.lambda*h_lm - obj.g);
                        obj.r0 = obj.r;
                        obj.J0 = obj.J;
                        obj.copyStateToBuffer(); % Populates x0 with current guess of parameters
                        obj.updateParameters(h_lm);
                        return
                    end
                    
                else
                    % Any subsequent iteration
                    ro = (0.5*obj.r0'*obj.r0-0.5*obj.r'*obj.r)/obj.L0;
                    if ro>0
                        obj.J0 = obj.J;
                        obj.r0 = obj.r;
                        obj.copyStateToBuffer(); % Populates x0
                        A = obj.J0'*obj.J0;
                        obj.g = obj.J0'*obj.r0; % Note no negative sign here
                        if norm(obj.g, 'inf') <= obj.eps1
                            success = 1;
                            return;
                        end
                        obj.lambda
                        max(1/3, 1-(2*ro-1)^3)
                        obj.lambda = obj.lambda * max(1/3, 1-(2*ro-1)^3);
                        obj.nu = 2;
                        A = A + obj.lambda*eye(size(A));
                        A_sparse = sparse(A);
                        L = chol(A_sparse,'lower');
                        h_lm = (L'\(L\-obj.g));
                        obj.L0 = 0.5*h_lm'*(obj.lambda*h_lm - obj.g);
                        update_delta = h_lm;
                        if norm(h_lm) <= obj.eps2*(norm(obj.getState()) + obj.eps2)
                            success = true;
                            return;
                        end
                        obj.updateParameters(h_lm);
                    else
                        obj.lambda = obj.lambda * obj.nu;
                        obj.nu = 2*obj.nu;
                        A = obj.J0'*obj.J0;
                        obj.g = obj.J0'*obj.r0; % Note no negative sign here
                        A = A + obj.lambda*eye(size(A));
                        A_sparse = sparse(A);
                        L = chol(A_sparse,'lower');
                        h_lm = (L'\(L\-obj.g));
                        obj.L0 = 0.5*h_lm'*(obj.lambda*h_lm - obj.g);
                        update_delta = h_lm;
                        if norm(h_lm) <= obj.eps2*(norm(obj.getState()) + obj.eps2)
                            success = true;
                            return;
                        end
                        obj.copyBufferToState();
                        obj.updateParameters(h_lm);
                    end
                end
                
            elseif strcmp(obj.opt_params.opt_type, 'LM2')
                % This code has been based off Algorithm 3.16 in the document
                % "Methods for non-linear least squares problems"
                % f(x) = obj.r = residual -> Page 2
                % mu = obj.lambda
                % Reset the success flag
                success = false;
                
                % First iteration
                if ~obj.LM_init_flag
                    
                    obj.LM_init_flag = 1;
                    
                    % Create hessian and gradient
                    A = obj.J'*obj.J;
                    obj.g = obj.J'*obj.r; % Note no negative sign here
                    
                    % Check if gradient is less than eps1
                    if norm(obj.g, 'inf') <= obj.eps1
                        success = 1;
                        update_delta = zeros(size(A,2),1);
                        return;
                    end
                    
                    % initialize lambda (according to algorithm, lambda = mu)
                    obj.lambda = obj.tau*max(diag(A));
                    
                    % Solve the system
                    A = A + obj.lambda*eye(size(A));
                    h_lm = getUpdateDelta(A, -obj.g, 1);
                    update_delta = h_lm;
                    
                    % Check for success based on update vector
                    if norm(h_lm) <= obj.eps2*(norm(obj.getState()) + obj.eps2)
                        success = true;
                    else
                        obj.L0 = 0.5*h_lm'*(obj.lambda*h_lm - obj.g); % This is L(0)-L(h_lm) from page 25
                        obj.r0 = obj.r;
                        obj.J0 = obj.J;
                        obj.copyStateToBuffer(); % Populates x0 with current guess of parameters (before updating params)
                        obj.updateParameters(h_lm);
                    end
                    
                else
                    % Any subsequent iteration
                    % The numerator in the below equation is from Equation
                    % 3.1b definition of F(x)
                    ro = (0.5*obj.r0'*obj.r0-0.5*obj.r'*obj.r)/obj.L0; 
                    if ro>0
                        % Step has been accepted, populate old variables
                        % with the new values
                        obj.J0 = obj.J;
                        obj.r0 = obj.r;
                        
                        % Copy the new state to the buffer and replace the
                        % old one
                        obj.copyStateToBuffer(); 
                        
                        A = obj.J'*obj.J;
                        obj.g = obj.J'*obj.r; % Note no negative sign here
                        
                        if norm(obj.g, 'inf') <= obj.eps1
                            success = 1;
                            update_delta = zeros(size(A,2),1);
                            return;
                        end
                        
                        % Update lambda and nu
                        obj.lambda = obj.lambda * max(1/3, 1-(2*ro-1)^3);
                        obj.nu = 2;
                    else
                        % Step was rejected, so use values from buffer 
                        A = obj.J0'*obj.J0;
                        obj.g = obj.J0'*obj.r0; % Note no negative sign here
                        
                        % We copy the buffer back to the state because the
                        % update has to be added to the old variables
                        obj.copyBufferToState();
                        
                        % Update lambda and nu
                        obj.lambda = obj.lambda * obj.nu;
                        obj.nu = 2*obj.nu;
                        
                    end
                    
                    A = A + obj.lambda*eye(size(A));
                    h_lm = getUpdateDelta(A, -obj.g, 1);
                    update_delta = h_lm;
                    obj.L0 = 0.5*h_lm'*(obj.lambda*h_lm - obj.g);
                    if norm(h_lm) <= obj.eps2*(norm(obj.getState()) + obj.eps2)
                        success = true;
                        return;
                    end
                    obj.updateParameters(h_lm);
                    
                end
            end
            
            %opts.SYM = true;
            %opts.POSDEF = true;
            %H = H + 0.0001*eye(size(H));
            %update_delta = linsolve(H, obj.g, opts);
        end
        
        function update_delta = getUpdateDelta(A, b, option_flag)
            if option_flag == 1
                A_sparse = sparse(A);
                L = chol(A_sparse,'lower');
                update_delta = (L'\(L\b));
            elseif option_flag == 2
                update_delta = A\b;
            end
        end
        
        function copyStateToBuffer(obj)
            pl = obj.parameter_container.parameter_list;
            for i=1:length(pl)
                if isa(pl{i}.parameter,'Transformation')
                    obj.x0{i} = pl{i}.parameter.matrix;
                else
                    obj.x0{i} = pl{i}.parameter.value;
                end
            end
        end
        
        function copyBufferToState(obj)
            for i=1:length(obj.parameter_container.parameter_list)
                if isa(obj.parameter_container.parameter_list{i}.parameter,'Transformation')
                    obj.parameter_container.parameter_list{i}.parameter.matrix = obj.x0{i};
                else
                    obj.parameter_container.parameter_list{i}.parameter.value = obj.x0{i};
                end
            end
        end
        
        function state_vec = getState(obj)
            pl = obj.parameter_container.parameter_list;
            state_vec = [];
            for i=1:length(pl)
                if isa(pl{i}.parameter,'Transformation')
                    x_param = tran2vec2(pl{i}.parameter.matrix);
                else
                    x_param = pl{i}.parameter.value;
                end
                state_vec = [state_vec x_param'];
            end
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

