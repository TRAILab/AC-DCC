function dsc_obj = loadTransformsAndTarget(data_files, dsc_obj)
%% This file loads the transforms from base to world and target points as
% necessary

global orig_T_WB;

% Load transformations required for visualization
dsc_obj.transforms = loadTransforms(data_files.transforms_file_path);

orig_T_WB = dsc_obj.transforms.world_T_base;

% Decide if we want to use random points all around the camera instead of a target
if(data_files.use_random_pts)
    target_pts = load(strcat(data_files.folder_path, data_files.target_pts_filename));
    dsc_obj.target_pts = target_pts.target_pts;
    
    % We assume that any environment that is not the checkerboard or
    % aprilgrid target, will have the same origin frame as the world frame
    % (TODO: Let user specify this)
    dsc_obj.transforms.world_T_target = eye(4);
    dsc_obj.target_pts_world = applyTransform(dsc_obj.transforms.world_T_target, dsc_obj.target_pts);

else 
    % Create target
    dsc_obj.target_pts = loadTargetPts(data_files.target_file_path);
    dsc_obj.target_pts_world = applyTransform(dsc_obj.transforms.world_T_target, dsc_obj.target_pts);

end