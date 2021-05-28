classdef MDHParamBeta<handle
    %TRANSFORMATIONPARAMETER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Internal storage
        value;
    end
    
    methods
       
         function [obj] = MDHParamBeta(init_value)
            obj.value = init_value;
        end
       
        function [] = manifoldPlus(obj,perturbation)
            obj.value = obj.value + perturbation;
        end
        
        %function [J_beta] = getJacobian(obj,d,r,alpha,input_vector )
        %    [ J_beta ] = thetaJacobian( obj.value, d, r,alpha,input_vector );
        %end
    end
end
