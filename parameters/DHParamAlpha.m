classdef DHParamAlpha<handle
    
    properties
        value;
    end
    
    methods
        
        function [obj] = DHParamAlpha(init_value)
            obj.value = init_value;
        end
        
        function [] = manifoldPlus(obj,perturbation)
            obj.value = obj.value + perturbation;
        end
        
        function [J_alpha] = getJacobian(obj, theta, d, r, alpha)
            all_jacobians = DHJacobians(theta, d, r, obj.value);
            J_alpha = all_jacobians(:,4);
        end
    end
end
