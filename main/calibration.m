clear variables
clc
close all
fclose('all');

% This is the main file to run to perform calibration

format long

%% Variable setup #################################### <--- Important to go through each of these and modify the values
reprojection_threshold = 2.0;           % Allowed reprojection threshold to decide if a measurement is good or not
axis_len = 0.4;                         % Length of the axis (for display purposes)
pixel_noise.std_dev = 0.0;              % Pixel noise std dev
pixel_noise.mean = 0;                   % Pixel noise mean
optimize_theta_flag_vec = [1 1 1];      % Which angles will be added with noise (0: no noise added, 1: noise added)
encoder_noise.mean = 0;                 % Encoder noise mean (deg)
encoder_noise.std_dev = 10;             % How much noise to add to the encoder values (deg)
encoder_noise.bounds = [-7 7];          % Noise to add on the encoders (not sure if we need it)
use_random_pts = 1;                     % Use random points in the environment (1) or a target (0) 
move_base = 0;                          % Decide if you want to move the drone
multicam = 1;
add_identity_residual = 0;              % This is the loop residual for all static cameras
measurement_vec = 1:30;                 % What measurements do we want to analyze
bad_meas_idxs = [];                     % If we know any measurements are bad
eval_flag = 0;                          % This is if we are doing evaluation and not the calibration
optimize_scale_offset = 0;              % We dont ever use this
pixel_error_formulation = 1;            % Is this pixel error formulation or pose loop formulation
real_image_mapping = containers.Map;    % Containers for real image mapping
show_real_world_images = 0;             % Show the pixel error on real world images
prior_joint_angles_vec = [0 0 0];       % Note this vec is a subset of optimize_theta_flag_vec, so it cant contain a 1 where a 0 is present above
encoder_std_dev_deg = 10500;            % Uncertainty on the joint angle (This is just a high random value ?)
have_true_values = 0;                   % Do we have the true value ? (This i sset to 1 if we are analyzing data from simulation)
show_residual_values = 1;
have_true_encoder_values = 0;

% Data location #################################### <--- Important to go through each of these and modify the values
data_files.folder_path = 'data/test_3_cam_new/';
data_files.measurement_type = 'train/';
data_files.real_image_path = 'real_images/'; % This is combined with folder path
data_files.camera_param_file_path = strcat(data_files.folder_path,'cameraParams.txt');
data_files.transforms_file_path = strcat(data_files.folder_path,'transforms.txt');
data_files.target_file_path = strcat(data_files.folder_path,'targetParams.txt');
data_files.sensors_file_path = strcat(data_files.folder_path,'sensor_param.txt');
data_files.calibration_params_file_path = strcat(data_files.folder_path,'minimalparam_init.txt'); % Use the initialization file for calibration
data_files.true_encoder_angles_path = strcat(data_files.folder_path,data_files.measurement_type, 'true_encoder_angles.txt');
true_params_path = strcat(data_files.folder_path, 'trueParams.txt');
data_files.use_random_pts = use_random_pts;

%% Initialize simulation object
dcc_obj = initDCC(data_files);
dcc_obj.reprojection_threshold = reprojection_threshold;
dcc_obj.data_files = data_files;
dcc_obj.eval_flag = eval_flag;
dcc_obj.optimize_scale_offset = optimize_scale_offset;
dcc_obj.optimize_theta_flag = sum(optimize_theta_flag_vec)>0;
dcc_obj.optimize_theta_flag_vec = optimize_theta_flag_vec;
dcc_obj.multicam = multicam;
dcc_obj.add_identity_residual = add_identity_residual;
dcc_obj.pixel_error_formulation = pixel_error_formulation;
dcc_obj.bad_meas_idxs = bad_meas_idxs;
dcc_obj.real_image_mapping = real_image_mapping;
dcc_obj.show_real_world_images = show_real_world_images;
dcc_obj.add_prior_joint_angles = sum(prior_joint_angles_vec)>0;
dcc_obj.prior_joint_angles_vec = prior_joint_angles_vec;
dcc_obj.encoder_std_dev_deg = encoder_std_dev_deg;
dcc_obj.encoder_std_dev_rad = deg2rad(encoder_std_dev_deg);
dcc_obj.have_true_values = have_true_values;
dcc_obj.show_residual_values = show_residual_values;
dcc_obj.have_true_encoder_values = have_true_encoder_values;
dcc_obj.use_random_points = use_random_pts;

% If we have the true encoder angles
if dcc_obj.have_true_encoder_values
    dcc_obj.true_encoder_angles = readmatrix(data_files.true_encoder_angles_path);
end

% Get measurements
[measurement_set, dcc_obj] = loadMeasurements(measurement_vec, dcc_obj);

%%% Now we need to set up the optimization parameters %%%%
% Note that the order is from moving to static
opt_problem = setupOptimizationProblem(dcc_obj, measurement_set);

% Setup optimizer params
opt_params.gradient_norm_threshold = 1e-12;
opt_params.step_norm_threshold = 1e-12;
opt_params.max_iterations = 500;
opt_params.success = 0;

% Show simulation object
% figure(5);
% clf;
dcc_obj.caz = 0;
dcc_obj.cel = 0;
disp('Showing DCC configuration at [0 0 0] angles');
[~] = displaySimulationObject(dcc_obj, [0 0 0], opt_problem);
[caz,cel] = view;
dcc_obj.caz = caz;
dcc_obj.cel = cel;

 % Optimize for Calibration Parameters
[dcc_obj, opt_problem] = calibrateMechanism(dcc_obj, opt_problem, opt_params, measurement_set);