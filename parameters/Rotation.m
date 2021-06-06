classdef Rotation<handle
    %TRANSFORMATIONPARAMETER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Internal storage
        matrix;
    end
    
    methods
        
        function [obj] = Rotation(rot_params)
            if size(rot_params,1) == 3 && size(rot_params,2) == 3
                obj.matrix = rot_params;
            elseif size(rot_params,1) == 1 && size(rot_params,2) == 3
                obj.matrix = vec2rot(rot_params');
            end
        end
        
        function r = rotate(obj, input_vector)
            % Create homogenous vector
            r = obj.matrix*input_vector;
        end
        
        function [r,J_R,J_p] = rotateAndJacobian(obj,input_vector)
            
            r = obj.rotate(input_vector);
            % Jacobian wrt point is just the rotation matrix
            J_p = obj.matrix;
            % Jacobian wrt transformation params is transformed point
            % in skew symmetric form
            J_R = -skewSymmetricMatrix3(r);
        end
        
        function [Rc,J_L,J_R] = composeAndJacobian(obj,input_rotation)
            
            % RHS composition
            Rc = Rotation([0 0 0]);
            Rc.matrix = obj.matrix*input_rotation.matrix;
            J_L = eye(3);
            J_R = obj.matrix;
        end
        
        function [R_inverse,J_inverse] = inverseAndJacobian(obj)
            
            R_inverse = Rotation([0 0 0]);
            R_inverse.matrix =  obj.matrix';
            % Jacobian of inverse function
            J_inverse = -R_inverse.matrix;
        end
        
        function [difference] = manifoldMinus(obj,input_rotation)
            % compute the boxminus
            %difference = vectorFromSkewSymmetricMatrix3(logm(obj.matrix*input_rotation.matrix'));
            difference = rot2vec(obj.matrix*input_rotation.matrix');
        end
        
        function [difference] = manifoldMinusQuaternion(obj,input_rotation)
            %convert everything to quaternions
            q_obj = qGetQ(obj.matrix);
            q_in = qGetQ(input_rotation.matrix);
            q_diff = qMul(q_obj,qInv(q_in));
            
            %implement log for quaterions
            q0 = q_diff(1);
            qv = q_diff(2:end);
            difference = 2*atan2(norm(qv),q0)* ( qv / norm(qv));
        end
        
        function [difference, J_left, J_right] = manifoldMinusAndJacobian(obj,input_rotation)
            % compute the boxminus
           
            % compute the jacobian using chain rule
            % R1 \boxminus R2
            % = logm(R1*inv(R2))
            % J_left = J_logm * J_compose_R1
            % J_right = J_logm *J_compose_inv(R2)*J_inv_R2
            
            [R_inv_r,J_inv_r] = input_rotation.inverseAndJacobian();
            [R_composed,J_compose_l,J_compose_r] = obj.composeAndJacobian(R_inv_r);
            [~,J_log] = Rotation.logAndJacobian(R_composed);
            J_left = J_log*J_compose_l;
            J_right = J_log*J_compose_r*J_inv_r;
            
            difference = vectorFromSkewSymmetricMatrix3(logm(R_composed.matrix));
            
        end
        
        function [] = manifoldPlus(obj,perturbation)
            % rotation perturbation is in so3
            %Rp = expm(skewSymmetricMatrix3(perturbation(1:3)));
            Rp = vec2rot(reshape(perturbation,3,1));
            obj.matrix = Rp*obj.matrix;
        end
        
        
    end
    
    methods(Static)
        function [R,J] = expAndJacobian(input_vector)
            wx = skewSymmetricMatrix3(input_vector);
            wn = norm(input_vector);
            R = Rotation([0 0 0]);
            R.matrix = expm(wx);
            
            if(wn > sqrt(eps))
                J = eye(3) + ((1-cos(wn))*wx)./(wn^2) + ((wn - sin(wn))*wx*wx)./(wn^3);
            else
                J = eye(3) + 0.5*wx;
            end
            
        end
        
        function [l,J] = logAndJacobian(input_rotation)
            l = vectorFromSkewSymmetricMatrix3(logm(input_rotation.matrix));
            [~,Jexp] = Rotation.expAndJacobian(l);
            J = inv(Jexp);
        end
    end
    
end

