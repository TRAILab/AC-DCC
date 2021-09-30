clear all
clc
close all

%% Description
% This file is used to visualize the simulation by observing the change in
% the mechanism using angles provided by the sliders.
format long
global input_angles dsc_obj orig_T_WB;

%% Variable setup #################################### <--- Important to go through each of these and modify the values
reprojection_threshold = 1.5;           % Allowed reprojection threshold to decide if a measurement is good or not
axis_len = 0.4;                         % Length of the axis (for display purposes)
pixel_noise.mean = 0;                   % Pixel noise mean
pixel_noise.std_dev = 0.0;              % Pixel noise std dev
encoder_noise.mean = 0;                 % Encoder noise mean (deg)
encoder_noise.std_dev = 0;              % How much noise to add to the encoder values (deg)
use_random_pts = 0;                     % Use random points in the environment (1) or a target (0)
target_pts_filename = 'noisy_cube.mat';         % This is the file that stores the target points

% Data location #################################### <--- Important to go through each of these and modify the values
data_files.folder_path = 'data/ijrr/multicamera_test/';
data_files.sensors_file_path = strcat(data_files.folder_path,'sensorParams.txt');
data_files.transforms_file_path = strcat(data_files.folder_path,'transforms.txt');
data_files.target_file_path = strcat(data_files.folder_path,'targetParams.txt');
data_files.calibration_params_file_path = strcat(data_files.folder_path,'trueParams.txt');
data_files.use_random_pts = use_random_pts;
data_files.target_pts_filename = target_pts_filename;

% Initialize simulation object
dsc_obj = initDCC(data_files);
dsc_obj = loadTransformsAndTarget(data_files, dsc_obj);
dsc_obj.pixel_noise = pixel_noise;
dsc_obj.encoder_noise.std_dev = deg2rad(encoder_noise.std_dev);
dsc_obj.encoder_noise.mean = deg2rad(encoder_noise.mean);
dsc_obj.reprojection_threshold = reprojection_threshold;
dsc_obj.data_files = data_files;
dsc_obj.optimize_theta_flag = sum(dsc_obj.optimize_theta_flag_vec)>0;
dsc_obj.use_random_points = use_random_pts;

% Go through the links and find out the midpoint of the angles
joint_angle_count = 0;
for i=1:length(dsc_obj.link_struct)
    if strcmp(dsc_obj.link_struct(i).type,'mdh') || strcmp(dsc_obj.link_struct(i).type,'dh')
        joint_angle_count = joint_angle_count + 1;
        input_angles(joint_angle_count) = mean(dsc_obj.link_struct(i).bounds);
    end
end

% Show the object and pixels on the image plane and setup the UI
showObjAndPix(dsc_obj, input_angles);
setupUI(dsc_obj);