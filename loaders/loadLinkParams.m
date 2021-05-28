function link_struct = loadLinkParams(file_path)

% sets an initial guess and the mappings

% gets the initial guess vector
params_struct = readDSCParams(file_path);

num_DH_links = 0;
for i=1:length(params_struct)
    if ~isempty(params_struct(i).bounds) && length(params_struct(i).params) == 4
        num_DH_links = num_DH_links + 1;
    end
end

% Get dynamic camera to end effector or J3 depending on what
% parameterization used
index_offset = 0;
link_struct = [];
EE_param_MC = [];
link_struct(1).index_map = [-1 -1 -1 -1 -1 -1];
link_struct(1).default_values = zeros(1,6);

if strcmp(params_struct(1).type,'mdh')
    for i=1:6
        current_param = params_struct(1).params(i);
        if(i==1 || i==4 || i==5) % this is an angle param, convert to rads; theta, d, a, alpha, beta, y
            current_param = deg2rad(current_param);
        end
        if(params_struct(1).opt(i)==1)        
            EE_param_MC = [EE_param_MC current_param]; 
            index_offset = index_offset+1;
            link_struct(1).index_map(i) = index_offset;
        end
        % set the default value for this parameter.  To be used later when we
        % are performing the optimization.
        link_struct(1).default_values(i) = current_param;
    end
    link_struct(1).offset = deg2rad(params_struct(1).offset);
    link_struct(1).bounds = deg2rad(params_struct(1).bounds);
    link_struct(1).type = params_struct(1).type;
elseif strcmp(params_struct(1).type,'6dof')
    for i=1:6
        current_param = params_struct(1).params(i);
        if(i>=1 && i<=3) % this is an angle param, convert to rads;
            current_param = deg2rad(current_param);
        end
        if(params_struct(1).opt(i)==1)        
            EE_param_MC = [EE_param_MC current_param];
            index_offset = index_offset+1;
            link_struct(1).index_map(i) = index_offset;
        end
        % set the default value for this parameter.  To be used later when we
        % are performing the optimization.
        link_struct(1).default_values(i) = current_param;
    end
    link_struct(1).type = params_struct(1).type;
end
link_struct(1).cam_name = strcat('M');

% Pull out the variables we wish to optimize
Qn_param_Q1 = [];
for i=1:num_DH_links
    link_opt = params_struct(i+1).opt;
    link_dh = params_struct(i+1).params;
    link_index_map = [-1 -1 -1 -1];
    % set the default parameters
    link_struct(i+1).default_values = [deg2rad(link_dh(1)) link_dh(2) link_dh(3) deg2rad(link_dh(4))];
    if(link_opt(1)>0)
        current_param  = deg2rad(link_dh(1)); % theta, comes in as degrees
        link_struct(i+1).default_values(1) =current_param;
        Qn_param_Q1 = [Qn_param_Q1 current_param];
        index_offset = index_offset+1;
        link_index_map(1) = index_offset;
    end
    if(link_opt(2)>0)
        link_struct(i+1).default_values(2) = link_dh(2);
        Qn_param_Q1 = [Qn_param_Q1 link_dh(2)];
        index_offset = index_offset+1;
        link_index_map(2) = index_offset;
    end
    if(link_opt(3)>0)
        link_struct(i+1).default_values(3) = link_dh(3);
        Qn_param_Q1 = [Qn_param_Q1 link_dh(3)];
        index_offset = index_offset+1;
        link_index_map(3) = index_offset;
    end
    if(link_opt(4)>0)  % alpha, comes in as degrees
        current_param  = deg2rad(link_dh(4));
        link_struct(i+1).default_values(4) =current_param ;
        Qn_param_Q1 = [Qn_param_Q1 current_param];
        index_offset = index_offset+1;
        link_index_map(4) = index_offset;
    end
    link_struct(i+1).index_map = link_index_map;
    link_struct(i+1).offset = deg2rad(params_struct(i+1).offset);
    link_struct(i+1).bounds =  deg2rad(params_struct(i+1).bounds);
    link_struct(i+1).type = params_struct(i+1).type;
end

static_cam_count = 0;
for m=num_DH_links+1+1:length(params_struct)
    static_cam_count = static_cam_count + 1;
    if strcmp(params_struct(m).type,'4dof')
        % setup the s_param_b
        link_struct(end+1).index_map = [-1 -1 -1 -1];
        link_struct(end).default_values = zeros(1,4);
        SC_param_BF = [];
        for i=1:4
            current_param = params_struct(m).params(i);
            if(i>=1 && i<=2) % this is an angle param, convert to rads;
                current_param = deg2rad(current_param);
            end
            if(params_struct(m).opt(i)==1)
                SC_param_BF = [SC_param_BF current_param];
                index_offset = index_offset+1;
                link_struct(end).index_map(i) = index_offset;
            end
            link_struct(end).default_values(i) = current_param;
        end
        link_struct(end).type = params_struct(m).type;
    elseif strcmp(params_struct(m).type,'6dof')
        % setup the s_param_b
        link_struct(end+1).index_map = [-1 -1 -1 -1 -1 -1];
        link_struct(end).default_values = zeros(1,6);
        SC_param_BF = [];
        for i=1:6
            current_param = params_struct(m).params(i);
            if(i>=1 && i<=3) % this is an angle param, convert to rads;
                current_param = deg2rad(current_param);
            end
            if(params_struct(m).opt(i)==1)
                SC_param_BF = [SC_param_BF current_param];
                index_offset = index_offset+1;
                link_struct(end).index_map(i) = index_offset;
            end
            link_struct(end).default_values(i) = current_param;
        end
        link_struct(end).type = params_struct(m).type;
    end
    link_struct(end).cam_name = strcat('S',num2str(static_cam_count));
end

end

