classdef DHParamTheta<handle

    properties
        value;
    end
    
    methods
       
        function [obj] = DHParamTheta(init_value)
            obj.value = init_value;
        end
       
        function [] = manifoldPlus(obj,perturbation)
            obj.value = obj.value + perturbation;
        end
        
        function [J_theta] = getJacobian(obj, theta, d, r, alpha)
            all_jacobians = DHJacobians(obj.value, d, r, alpha);
            J_theta = all_jacobians(:,1);
        end
    end
end
