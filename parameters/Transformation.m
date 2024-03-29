classdef Transformation<handle
    %% This file is the transformation parameter class
    
    properties
        matrix;
    end
    
    methods
        
        function [obj] = Transformation(pose_params)
             % If pose_params is a vector
            if length(pose_params)==6
                obj.matrix = vec2tran2(pose_params');
            % If pose_params is a matrix
            elseif length(pose_params)==4
                obj.matrix = pose_params;
            else
                disp('Wrong input length');
            end
        end
        
        function r = transform(obj, input_vector)
            % Transforms a vector via the matrix
            input_vector = reshape(input_vector,[3,1]);
            input_vector = [input_vector ;1];
            r = obj.matrix*input_vector;
            r = r(1:3);
        end
        
        function [transformed, J_T, J_p] = transformAndJacobian(obj, input_vector)
            % Transforms the point an returns the Jacobians
            input_vector = reshape(input_vector,[3,1]);
            transformed = obj.transform(input_vector);
            transformed = reshape(transformed, [1,3]);
            % Jacobian calculation
            r = obj.matrix(1:3,1:3)*(input_vector);
            % Jacobian wrt transformation params is transformed point
            % in skew symmetric form
            % Original
            J_R = -skewSymmetricMatrix3(r);
            % Jacobian wrt translation is just I
            J_t = eye(3);
            J_T = [J_R J_t];
            % Jacobian wrt point is just the rotation matrix
            J_p = obj.matrix(1:3,1:3);
        end
        
        function [composed, J_obj, J_in] = composeAndJacobian(obj, input_transform)
            % Composes two transformations and returns the jacobians
            composed = Transformation(obj.matrix*input_transform.matrix);
            % Jacobian wrt the object
            J_obj = zeros(6,6);
            J_obj(1:3,1:3) = eye(3);
            t = obj.matrix(1:3,1:3)*input_transform.matrix(1:3,4);
            J_obj(4:6,1:3) = -skewSymmetricMatrix3(t);
            J_obj(4:6,4:6) = eye(3);
            
            % Jacobian wrt the input
            J_in = zeros(6,6);
            R = obj.matrix(1:3,1:3);
            J_in(1:3,1:3) = R;
            J_in(4:6,4:6) = R;
                    
        end
        
        function [T_inverse, J_inverse] = inverseAndJacobian(obj)
                % Returns the inverse transformation and the Jacobian
                T_inverse = Transformation([0 0 0 0 0 0]);
                R_inv = obj.matrix(1:3,1:3)';
                T_inverse.matrix(1:3,1:3) = R_inv;
                t = -R_inv*obj.matrix(1:3,4);
                T_inverse.matrix(1:3,4) = t;
                
                % Jacobian Calculation
                J_inverse = zeros(6,6);
                J_inverse(1:3,1:3) = -R_inv;
                J_inverse(4:6,1:3) = skewSymmetricMatrix3(t)*R_inv;
                J_inverse(4:6,4:6) = -R_inv;
        
        end
        
        function [difference] = manifoldMinus(obj, input_transformation)
            % Returns the manifold minus between two transformations
            % compute the boxminus for the rotations
            R_obj = Rotation(obj.matrix(1:3,1:3));
            R_in =  Rotation(input_transformation.matrix(1:3,1:3));
            diff_rot = R_obj.manifoldMinus(R_in);
            
            %transformation difference is just vector diff
            diff_trans = obj.matrix(1:3,4) - input_transformation.matrix(1:3,4);            
            difference = [diff_rot ; diff_trans];
           
        end
        
         function [difference] = manifoldMinusQuaternion(obj, input_transformation)
            % compute the boxminus for the rotations
            R_obj = Rotation([0 0 0 0 0 0]);
            R_obj.matrix = obj.matrix(1:3,1:3);
            R_in =  Rotation([0 0 0 0 0 0]);
            R_in.matrix = input_transformation.matrix(1:3,1:3);
            diff_rot = R_obj.manifoldMinusQuaternion(R_in);
            
            %transformation difference is just vector diff
            diff_trans = obj.matrix(1:3,4) - input_transformation.matrix(1:3,4);            
            difference = [diff_rot ; diff_trans];
           
        end
        
        function [difference, J_left, J_right] = manifoldMinusAndJacobian(obj, T_right)
            % Returns the manifold difference between two transformations
            % and the corresponding Jacobians
            difference = obj.manifoldMinus(T_right);
            %compute Jacobians
            R1 = Rotation(obj.matrix(1:3,1:3));
            R2 = Rotation(T_right.matrix(1:3,1:3));
            [~, J_R_left, J_R_right] = R1.manifoldMinusAndJacobian(R2);
            % Left Jacobian
            J_left = zeros(6,6);
            J_left(1:3,1:3) = J_R_left;
            J_left(4:6,4:6) = eye(3);
            % Right Jacobian
            J_right = zeros(6,6);
            J_right(1:3,1:3) = J_R_right;
            J_right(4:6,4:6) = -eye(3);
            
        end
        
        function [] = manifoldPlus(obj, perturbation)
            % Performs the manifold plus to the transformation matrix
            perturbation = reshape(perturbation,[6,1]);
            % rotation perturbation is in so3
            
            % Original
            Rp = expm(skewSymmetricMatrix3(perturbation(1:3)));
            obj.matrix(1:3,1:3) = Rp * obj.matrix(1:3,1:3);
            
            % translation perturbation is in R^3 (not exactly se3)
            tp = perturbation(4:6);
            obj.matrix(1:3,4) = obj.matrix(1:3,4) + tp;
        end
        
        function [p] = getParams(obj)
            % Gets the parameters from the transformation
            p = trans2param(obj.matrix);
        end
        
    end
    
end

