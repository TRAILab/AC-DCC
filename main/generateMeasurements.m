clear all
clc
close all

% This is the main file to run to generate measurements
% Please note: Any array of points is stored as a N x D array, where N is
% the number of points and D is the dimension. This makes it easier to
% write functions to process the points

format long

%% Variable setup #################################### <--- Important to go through each of these and modify the values
reprojection_threshold = 1.5;           % Allowed reprojection threshold to decide if a measurement is good or not
axis_len = 0.4;                         % Length of the axis (for display purposes)
pixel_noise.std_dev = 0.2;              % Pixel noise std dev
pixel_noise.mean = 0;                   % Pixel noise mean
pixel_noise.bounds = [-1.2 1.2];        % Noise bounds (not sure if we need it)
encoder_noise.mean = 0;                 % Encoder noise mean (deg)
encoder_noise.std_dev = 10;             % How much noise to add to the encoder values (deg)
encoder_noise.bounds = [-7 7];          % Noise to add on the encoders (not sure if we need it)
transformation_noise.trans.mean = 0;    % Transformation noise, translation
transformation_noise.trans.std_dev = 0.0;  
transformation_noise.rot.mean = 0;      % Transformation noise, rotation
transformation_noise.rot.std_dev = 0; 
optimize_theta_flag_vec = [0 0 0];      % Which angles will be added with noise (0: no noise added, 1: noise added)
use_random_pts = 1;                     % Use random points in the environment (1) or a target (0)
angle_type = 'random';                  % Type of angles we want, (linear, random)
num_random_angles = 30;                  % Number of random angles if random angle type
joint_angle_limits = [-170 170;           % Angle Limits from which to collect measurements (deg). This should be a Nx2, where N = num of joints 
                      -30 30; 
                      -120 40];        
num_linear_angles_per_joint = [4;4;5];  % Number of angles per joint to collect measurements if linear angle type. This should be an Nx1 where N = num of joints.   
move_base = 0;                          % Decide if you want to move the drone
evaluation_flag = 0;                    % Are we performing the evaluation of the calibration

% Data location #################################### <--- Important to go through each of these and modify the values
data_files.folder_path = 'data/test_3_cam_new/';
data_files.measurement_type = 'train/';
data_files.sensors_file_path = strcat(data_files.folder_path,'sensor_param.txt');
data_files.transforms_file_path = strcat(data_files.folder_path,'transforms.txt');
data_files.target_file_path = strcat(data_files.folder_path,'targetParams.txt');
data_files.calibration_params_file_path = strcat(data_files.folder_path,'minimalparam_true.txt');
data_files.use_random_pts = use_random_pts;

%% Initialize simulation object
sim_obj = initDCC(data_files);
sim_obj = loadTransformsAndTarget(data_files, sim_obj);
sim_obj.reprojection_threshold = reprojection_threshold;
sim_obj.axis_len = axis_len;
sim_obj.pixel_noise = pixel_noise;
sim_obj.encoder_noise.std_dev = deg2rad(encoder_noise.std_dev);
sim_obj.encoder_noise.mean = deg2rad(encoder_noise.mean);
sim_obj.encoder_noise.bounds = deg2rad(encoder_noise.bounds);
sim_obj.transformation_noise.trans = transformation_noise.trans;
sim_obj.transformation_noise.rot.mean = deg2rad(transformation_noise.rot.mean);
sim_obj.transformation_noise.rot.std_dev = deg2rad(transformation_noise.rot.std_dev);
sim_obj.data_files = data_files;
sim_obj.evaluation_flag = evaluation_flag;
sim_obj.num_collected_measurements = 0;
sim_obj.optimize_scale_offset = 0;
sim_obj.optimize_theta_flag = sum(optimize_theta_flag_vec)>0;
sim_obj.optimize_theta_flag_vec = optimize_theta_flag_vec;
sim_obj.move_base = move_base;
sim_obj.use_random_points = use_random_pts;

%% Generate angle set for linear spacing
if(strcmp('linear', angle_type)) 
    disp('Generating angle set for linear spacing:');
    
    joint_spacing = zeros(size(joint_angle_limits,1),1);
    for i=1:size(joint_angle_limits,1)
        joint_spacing(i) = (joint_angle_limits(i,2)-joint_angle_limits(i,1))/(num_linear_angles_per_joint(i)-1);
        joint_angles{i} = joint_angle_limits(i,1):joint_spacing(i):joint_angle_limits(i,2);
    end
    
    [a, b, c] = ndgrid(joint_angles{1}, joint_angles{2}, joint_angles{3});
    measurement_angle_set_deg = [a(:), b(:), c(:)];
    measurement_angle_set_rad = deg2rad(measurement_angle_set_deg);
end

%% Generate angle set for random spacing
if(strcmp('random', angle_type))
    disp('Generating angle set for random spacing:');
    
    measurement_angle_set_deg = zeros(num_random_angles, sim_obj.num_DH_links);
    for i=1:size(joint_angle_limits,1)
        measurement_angle_set_deg(:,i) = (joint_angle_limits(i,2)-joint_angle_limits(i,1)).*rand(num_random_angles,1) + joint_angle_limits(i,1);
    end
    measurement_angle_set_rad = deg2rad(measurement_angle_set_deg);
end

%% Write the measurements
writeMeasurementsToFile(sim_obj, measurement_angle_set_rad);