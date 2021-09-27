classdef OptimizationParameter<handle
    
    %% Description
    % This file holds all the information for getting the values of a
    % particular parameter class
    
    properties
        % Parameter object (needs to have manifoldPlus implemented)
        parameter
        tangentspace_dim
        column_idx
        key
    end
    
    methods
        
        % Create a parameter for optimization
        function obj = OptimizationParameter(parameter, tangentspace_dim, key)
            if nargin>2
                obj.key = key;
            else
                obj.key = 'empty';
            end
            obj.parameter = parameter;
            obj.tangentspace_dim = tangentspace_dim;
        end
        
        % Set the column index based on the parameter
        function [] = setColumnIndex(obj,idx)
            obj.column_idx = idx;
        end
        
    end
    
end

