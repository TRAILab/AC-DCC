function link_struct = loadLinkParams(file_path)

% sets an initial guess and the mappings

% gets the initial guess vector
params_struct = loadDSCParams(file_path);

num_DH_links = 0;
for i=1:length(params_struct)
    if ~isempty(params_struct(i).bounds) && length(params_struct(i).params) == 4
        num_DH_links = num_DH_links + 1;
    end
end

% Get dynamic camera to end effector or J3 depending on what parameterization used
index_offset = 0;
link_struct = [];
link_struct(1).index_map = [-1 -1 -1 -1 -1 -1];
link_struct(1).default_values = zeros(1,6);

if strcmp(params_struct(1).type,'mdh')
    params = params_struct(1).params;
    link_struct(1).default_values = [deg2rad(params(1)) params(2) params(3) deg2rad(params(4)) deg2rad(params(5)) params(6)];
    link_struct(1).opt_theta = 0;
    for i=1:6
        if(params_struct(1).opt(i)==1)
            if i==1
                link_struct(1).opt_theta = 1;
            else
                index_offset = index_offset + 1;
                link_struct(1).index_map(i) = index_offset;
            end
        end
    end
    link_struct(1).offset = deg2rad(params_struct(1).offset);
    link_struct(1).bounds = deg2rad(params_struct(1).bounds);
    link_struct(1).type = params_struct(1).type;
elseif strcmp(params_struct(1).type,'6dof')
    params = params_struct(1).params;
    link_struct(1).default_values = [deg2rad(params(1)) deg2rad(params(2)) deg2rad(params(3)) params(4) params(5) params(6)];
    for i=1:6
        if(params_struct(1).opt(i)==1)        
            index_offset = index_offset+1;
            link_struct(1).index_map(i) = index_offset;
        end
    end
    link_struct(1).type = params_struct(1).type;
end
link_struct(1).cam_name = strcat('M');

% Pull out the variables we wish to optimize
for i=1:num_DH_links
    link_opt = params_struct(i+1).opt;
    params = params_struct(i+1).params;
    link_index_map = [-1 -1 -1 -1];
    link_struct(i+1).default_values = [deg2rad(params(1)) params(2) params(3) deg2rad(params(4))];
    link_struct(i+1).opt_theta = 0;
    for j=1:4
        if link_opt(j)==1
            if j==1
                link_struct(i+1).opt_theta = 1;
            else
                index_offset = index_offset + 1;
                link_index_map(j) = index_offset;
            end
        end
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
        link_struct(end+1).index_map = [-1 -1 -1 -1];
        params = params_struct(m).params;
        link_struct(end).default_values = [deg2rad(params(1)) deg2rad(params(2)) params(3) params(4)];
        for i=1:4
            if(params_struct(m).opt(i)==1)
                index_offset = index_offset + 1;
                link_struct(end).index_map(i) = index_offset;
            end
        end
        link_struct(end).type = params_struct(m).type;
    elseif strcmp(params_struct(m).type,'6dof')
        link_struct(end+1).index_map = [-1 -1 -1 -1 -1 -1];
        params = params_struct(m).params;
        link_struct(end).default_values = [deg2rad(params(1)) deg2rad(params(2)) deg2rad(params(3)) params(4) params(5) params(6)];
        for i=1:6
            if(params_struct(m).opt(i)==1)
                index_offset = index_offset + 1;
                link_struct(end).index_map(i) = index_offset;
            end
        end
        link_struct(end).type = params_struct(m).type;
    end
    link_struct(end).cam_name = strcat('S',num2str(static_cam_count));
end

end


