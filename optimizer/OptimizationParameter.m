classdef OptimizationParameter<handle
    %TRASNFORMATIONPARAMETER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Parameter object (needs to have manifoldPlus implemented)
        parameter
        tangentspace_dim
        column_idx
        key
    end
    
    methods
        function obj = OptimizationParameter(parameter,tangentspace_dim,key)
            if nargin>2
                
                obj.key = key;
            else
                obj.key = 'empty';
            end
            obj.parameter = parameter;
            obj.tangentspace_dim = tangentspace_dim;
        end
        
        function [] = setColumnIndex(obj,idx)
            obj.column_idx = idx;
        end
        
    end
    
end

