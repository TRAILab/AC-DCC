classdef DHParamAlpha<handle
    %TRANSFORMATIONPARAMETER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Internal storage
        value;
    end
    
    methods
        
        function [obj] = DHParamAlpha(init_value)
            obj.value = init_value;
        end
        
        
        function [] = manifoldPlus(obj,perturbation)
            obj.value = obj.value+ perturbation;
        end
        
        function [J_alpha] = getJacobian(obj, theta, d, r,input_vector )
            [ J_alpha ] = alphaJacobian( theta, d, r,obj.value,input_vector );
        end
    end
end
