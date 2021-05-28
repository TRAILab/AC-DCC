function dcc_obj = initDCC(data_files)

%% Initialize simulation object
dcc_obj.cameras = loadCameraData(data_files);

% Load calibration parameters
link_struct = loadLinkParams(data_files.calibration_params_file_path);
dcc_obj.link_struct = link_struct;

dcc_obj.use_modified_DH_flag = 0;
dcc_obj.use_4dof_flag = 0;
for i=1:length(link_struct)
    if strcmp(dcc_obj.link_struct(i).type,'mdh')
        dcc_obj.use_modified_DH_flag = 1;
    end
    if strcmp(dcc_obj.link_struct(i).type,'4dof')
        dcc_obj.use_4dof_flag = dcc_obj.use_4dof_flag + 1;
    end
end

% Store the number of DH links in the simulation object
dcc_obj.num_DH_links = 0;
for i=1:length(link_struct)
    if strcmp(link_struct(i).type,'dh')
        dcc_obj.num_DH_links = dcc_obj.num_DH_links + 1;
    end
end

assert(dcc_obj.num_DH_links + length(dcc_obj.cameras) == length(link_struct), 'There is a mismatch between number of links and camera parameters');