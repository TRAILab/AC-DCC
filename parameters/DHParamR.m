classdef DHParamR<handle
    %TRANSFORMATIONPARAMETER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Internal storage
        value;
    end
    
    methods
        
         function [obj] = DHParamR(init_value)
            obj.value = init_value;
        end
        
        function [] = manifoldPlus(obj,perturbation)
            obj.value = obj.value+ perturbation;
        end
        
        function [J_r] = getJacobian(obj,theta, d,alpha,input_vector )
            [ J_r ] = rJacobian( theta, d, obj.value,alpha,input_vector );
        end
    end
end
