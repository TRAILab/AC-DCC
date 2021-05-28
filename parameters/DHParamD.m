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
        
        function [J_d] = getJacobian(obj,theta, r,alpha,input_vector )
            [ J_d ] = dJacobian(theta, obj.value, r, alpha, input_vector);
        end
    end
end
