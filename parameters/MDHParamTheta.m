classdef MDHParamTheta<handle
    %TRANSFORMATIONPARAMETER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Internal storage
        value;
    end
    
    methods
       
         function [obj] = MDHParamTheta(init_value)
            obj.value = init_value;
        end
       
        function [] = manifoldPlus(obj,perturbation)
            obj.value = wrapToPi(obj.value + perturbation);
        end
        
    end
end
