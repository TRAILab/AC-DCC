classdef DHParamR<handle
    
    properties
        value;
    end
    
    methods
        
        function [obj] = DHParamR(init_value)
            obj.value = init_value;
        end
        
        function [] = manifoldPlus(obj,perturbation)
            obj.value = obj.value + perturbation;
        end
        
        function [J_r] = getJacobian(obj, theta, d, r, alpha)
            all_jacobians = DHJacobians(theta, d, obj.value, alpha);
            J_r = all_jacobians(:,3);
        end
    end
end
