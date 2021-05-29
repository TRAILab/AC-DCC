classdef DHParamD<handle

    properties
        value;
    end
    
    methods
        
        function [obj] = DHParamD(init_value)
            obj.value = init_value;
        end
        
        function [] = manifoldPlus(obj,perturbation)
            obj.value = obj.value + perturbation;
        end
        
        function [J_d] = getJacobian(obj, theta, d, r, alpha)
            all_jacobians = DHJacobians(theta, obj.value, r, alpha);
            J_d = all_jacobians(:,2);
        end
    end
end
