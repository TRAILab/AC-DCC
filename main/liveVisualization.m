clear all
clc
close all

% This file allows to visualize the simulation object and projections onto
% the image plane for each camera. It allows for modifying joint angle
% values with sliders.

global input_angles dsc_obj comb_fig orig_T_WB;

%% Variable setup #################################### <--- Important to go through each of these and modify the values
reprojection_threshold = 1.5;           % Allowed reprojection threshold to decide if a measurement is good or not
axis_len = 0.4;                         % Length of the axis (for display purposes)
pixel_noise.std_dev = 0.2;              % Pixel noise std dev
pixel_noise.mean = 0;                   % Pixel noise mean
pixel_noise.bounds = [-1.2 1.2];        % Noise bounds (not sure if we need it)
optimize_theta_flag_vec = [0 0 0];      % Which angles will be added with noise (0: no noise added, 1: noise added)
encoder_noise.mean = 0;                 % Encoder noise mean (deg)
encoder_noise.std_dev = 10;             % How much noise to add to the encoder values (deg)
encoder_noise.bounds = [-7 7];          % Noise to add on the encoders (not sure if we need it)
use_random_pts = 0;                     % Use random points in the environment (1) or a target (0)
angle_type = 'random';                  % Type of angles we want, (linear, random)
num_random_angles = 70;                 % Number of random angles
joint_angle_limits = [-100 100;         % Angle Limits from which to collect measurements (deg). This should be a Nx2, where N = num of joints 
                      -70 70; 
                      -200 200];        
num_linear_angles_per_joint = [3;4;5];  % Number of angles per joint to collect measurements. This should be an Nx1 where N = num of joints.   
move_base = 0;                          % Decide if you want to move the drone
evaluation_flag = 0;                    % Are we performing the evaluation of the calibration

% Data location #################################### <--- Important to go through each of these and modify the values
data_files.folder_path = 'data/test_3_cam/';
data_files.measurement_type = 'train/';
data_files.sensors_file_path = strcat(data_files.folder_path,'sensor_param.txt');
data_files.transforms_file_path = strcat(data_files.folder_path,'transforms.txt');
data_files.target_file_path = strcat(data_files.folder_path,'targetParams.txt');
data_files.calibration_params_file_path = strcat(data_files.folder_path,'minimalparam_true.txt');
data_files.use_random_pts = use_random_pts;

% Initialize simulation object
dsc_obj = initDCC(data_files);
dsc_obj = loadTransformsAndTarget(data_files, dsc_obj);
dsc_obj.pixel_noise = pixel_noise;
dsc_obj.encoder_noise.std_dev = deg2rad(encoder_noise.std_dev);
dsc_obj.encoder_noise.mean = deg2rad(encoder_noise.mean);
dsc_obj.encoder_noise.bounds = deg2rad(encoder_noise.bounds);
dsc_obj.reprojection_threshold = reprojection_threshold;
dsc_obj.data_files = data_files;
dsc_obj.evaluation_flag = evaluation_flag;
dsc_obj.num_collected_measurements = 0;
dsc_obj.optimize_scale_offset = 0;
dsc_obj.optimize_theta_flag = sum(optimize_theta_flag_vec)>0;
dsc_obj.optimize_theta_flag_vec = optimize_theta_flag_vec;
dsc_obj.move_base = move_base;
dsc_obj.use_random_points = use_random_pts;

% Input angles all at 0 degrees
joint_angle_count = 0;
for i=1:length(dsc_obj.link_struct)
    if strcmp(dsc_obj.link_struct(i).type,'mdh') || strcmp(dsc_obj.link_struct(i).type,'dh')
        joint_angle_count = joint_angle_count + 1;
        input_angles(joint_angle_count) = mean(dsc_obj.link_struct(i).bounds);
    end
end

% Initial display
showObjAndPix(dsc_obj, input_angles);

% Setup UI
setupUI(dsc_obj);
