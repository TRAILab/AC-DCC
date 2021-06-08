function dcc_obj = initDCC(data_files)

%% Initialize simulation object

% Load the intrinsics for each camera
dcc_obj.cameras = loadCameraData(data_files);

% % Load transformations
% dcc_obj.transforms = loadTransforms(data_files.transforms_file_path);
% 
% % Create target
% dcc_obj.target_pts = loadTargetPts(data_files.target_file_path);
% dcc_obj.target_pts_world = applyTransform(dcc_obj.transforms.world_T_target,dcc_obj.target_pts);
% 
% % Decide if we want to use random points all around the camera instead of
% % a target
% if(data_files.use_random_pts)
%     target_pts = load(strcat(data_files.folder_path,'target_pts.mat'));
%     dcc_obj.target_pts = target_pts.target_pts;
%     dcc_obj.transforms.world_T_target = eye(4);
%     dcc_obj.target_pts_world = applyTransform(dcc_obj.transforms.world_T_target, dcc_obj.target_pts);
% end

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
optimize_theta_flag_vec = [];
for i=1:length(link_struct)
    if strcmp(link_struct(i).type,'dh')
        dcc_obj.num_DH_links = dcc_obj.num_DH_links + 1;
    end
    optimize_theta_flag_vec = [optimize_theta_flag_vec link_struct(i).opt_theta];
end
dcc_obj.optimize_theta_flag_vec = optimize_theta_flag_vec;

assert(dcc_obj.num_DH_links + length(dcc_obj.cameras) == length(link_struct), 'There is a mismatch between number of links and camera parameters');