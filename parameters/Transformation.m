classdef Transformation<handle
    %TRANSFORMATIONPARAMETER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        matrix;
    end
    
    methods
        
        function [obj] = Transformation(pose_params)
            obj.matrix = vec2tran(pose_params');
        end
        
        function r = transform(obj, input_vector)
            input_vector = reshape(input_vector,[3,1]);
            input_vector = [input_vector ;1];
            r = obj.matrix*input_vector;
            r = r(1:3);
        end
        
        function [transformed, J_T, J_p] = transformAndJacobian(obj,input_vector)
        
            input_vector = reshape(input_vector,[3,1]);
            transformed = obj.transform(input_vector);
            
            % Jacobian calculation
            r = obj.matrix(1:3,1:3)*(input_vector);
            J_R = -skewSymmetricMatrix3(r);
            J_t = eye(3);
            J_T = [J_R J_t];
            
            % Jacobian wrt point is just the rotation matrix
            J_p = obj.matrix(1:3,1:3);
        end
        
        function [composed, J_obj, J_in] = composeAndJacobian(obj,input_transform)
           
            composed = Transformation([0 0 0 0 0 0]);
            composed.matrix = obj.matrix*input_transform.matrix;
            % Jacobian wrt the object
            J_obj = zeros(6,6);
            J_obj(1:3,1:3) = eye(3);
            t = obj.matrix(1:3,1:3)*input_transform.matrix(1:3,4);
            J_obj(4:6,1:3) = -skewSymmetricMatrix3(t);
            J_obj(4:6,4:6) = eye(3);
            
            %Jacobian wrt the input
            J_in = zeros(6,6);
            R = obj.matrix(1:3,1:3);
            J_in(1:3,1:3) = R;
            J_in(4:6,4:6) = R;
                    
        end
        
        function [T_inverse, J_inverse] = inverseAndJacobian(obj)
        
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
            % compute the boxminus for the rotations
            R_obj = Rotation([0 0 0]);
            R_obj.matrix = obj.matrix(1:3,1:3);
            R_in =  Rotation([0 0 0]);
            R_in.matrix = input_transformation.matrix(1:3,1:3);
            diff_rot = R_obj.manifoldMinus(R_in);
            
            %transformation difference is just vector diff
            diff_trans = obj.matrix(1:3,4) - input_transformation.matrix(1:3,4);            
            difference = [diff_rot ; diff_trans];
           
        end
        
        function [difference, J_left, J_right] = manifoldMinusAndJacobian(obj, T_right)
            difference = obj.manifoldMinus(T_right);
            %compute Jacobians
            R1 = Rotation([0 0 0]);
            R2 = Rotation([0 0 0]);
            R1.matrix = obj.matrix(1:3,1:3);
            R2.matrix = T_right.matrix(1:3,1:3);
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
            perturbation = reshape(perturbation,[6,1]);
            % rotation perturbation is in so3
            
            % Original
            Rp = expm(skewSymmetricMatrix3(perturbation(1:3)));
            obj.matrix(1:3,1:3) = Rp * obj.matrix(1:3,1:3);
            
            % translation perturbation is in R^3 (not exactly se3)
            tp = perturbation(4:6);
            obj.matrix(1:3,4) = obj.matrix(1:3,4) + tp;
        end
        
    end
end

