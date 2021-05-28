function dsc_obj = loadTransformsAndTarget(data_files, dsc_obj)

global orig_T_WB;

% Load transformations
dsc_obj.transforms = loadTransforms(data_files.transforms_file_path);

orig_T_WB = dsc_obj.transforms.world_T_base;

% Create target
dsc_obj.target_pts = loadTargetPts(data_files.target_file_path);
dsc_obj.target_pts_world = applyTransform(dsc_obj.transforms.world_T_target, dsc_obj.target_pts);

% Decide if we want to use random points all around the camera instead of
% a target
if(data_files.use_random_pts)
    target_pts = load(strcat(data_files.folder_path,'target_pts.mat'));
    dsc_obj.target_pts = target_pts.target_pts;
    dsc_obj.transforms.world_T_target = eye(4);
    dsc_obj.target_pts_world = applyTransform(dsc_obj.transforms.world_T_target, dsc_obj.target_pts);
end