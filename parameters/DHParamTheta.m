classdef DHParamTheta<handle
    %TRANSFORMATIONPARAMETER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Internal storage
        value;
    end
    
    methods
       
         function [obj] = DHParamTheta(init_value)
            obj.value = init_value;
        end
       
        function [] = manifoldPlus(obj,perturbation)
            obj.value = obj.value+ perturbation;
        end
        
        function [J_theta] = getJacobian(obj,d,r,alpha,input_vector )
            [ J_theta ] = thetaJacobian( obj.value, d, r,alpha,input_vector );
        end
    end
end
