classdef ParameterContainer<handle
    
    %% Description
    % This holds the parameter container for all the optimization variables
    
    properties
        parameter_list = {};
        index_counter = 1;
        parameter_key_map = containers.Map;
    end
    
    methods
        function [] = addParameter(obj,parameter)
           parameter.setColumnIndex(obj.index_counter)
           obj.index_counter = obj.index_counter+parameter.tangentspace_dim;
           
           % Add to parameter list
           obj.parameter_list{end+1} = parameter;
           
           % Keep track of key index mapping
           obj.parameter_key_map(parameter.key) =  length(obj.parameter_list);
        end
        
        function [idx] = getKeyIndex(obj,key)
            if(isKey(obj.parameter_key_map,key))
                idx = obj.parameter_key_map(key);
            else
                error('parameter_container::key not found!');
            end
        end
        
        function [value] = getKeyValue(obj,key)
            if isKey(obj.parameter_key_map, key)
                idx = obj.parameter_key_map(key);
                if isa(obj.parameter_list{idx}.parameter,'Transformation')
                    value = obj.parameter_list{idx}.parameter.matrix;
                else
                    value = obj.parameter_list{idx}.parameter.value;
                end
            else
                error('parameter_container::key not found!');
            end
        end
        
    end
    
end

