function dcc_obj = initDCC(data_files)
%% Initializes the Dynamic Camera Cluster

% Load the intrinsics for each camera
dcc_obj.cameras = loadCameraData(data_files);

% Load calibration parameters
link_struct = loadLinkParams(data_files.calibration_params_file_path);
dcc_obj.link_struct = link_struct;

% Determine what parameterization we are using and which joint angles to
% optimize over.
dcc_obj.use_modified_DH_flag = 0;
dcc_obj.use_4dof_flag = 0;
dcc_obj.num_DH_links = 0;
optimize_theta_flag_vec = [];
for i=1:length(link_struct)
    if strcmp(link_struct(i).type,'mdh')
        dcc_obj.use_modified_DH_flag = 1;
    end
    if strcmp(link_struct(i).type,'4dof')
        dcc_obj.use_4dof_flag = dcc_obj.use_4dof_flag + 1;
    end
    if strcmp(link_struct(i).type,'dh')
        dcc_obj.num_DH_links = dcc_obj.num_DH_links + 1;
    end
    optimize_theta_flag_vec = [optimize_theta_flag_vec link_struct(i).opt_theta];
end
dcc_obj.optimize_theta_flag_vec = optimize_theta_flag_vec;

% Checsk that the we have the right number of links
assert(dcc_obj.num_DH_links + length(dcc_obj.cameras) == length(link_struct), 'There is a mismatch between number of links and camera parameters');