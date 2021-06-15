clear variables
clc
close all
fclose('all');

% This is the main file to run to perform calibration

format long

%% Variable setup #################################### <--- Important to go through each of these and modify the values
reprojection_threshold = 2.0;           % Allowed reprojection threshold to decide if a measurement is good or not
axis_len = 0.4;                         % Length of the axis (for display purposes)
use_random_pts = 1;                     % Use random points in the environment (1) or a target (0) 
move_base = 0;                          % Decide if you want to move the drone
add_identity_residual = 0;              % This is the loop residual for all static cameras
measurement_vec = 1:31;                 % What measurements do we want to analyze
bad_meas_idxs = [];                     % If we know any measurements are bad
real_image_mapping = containers.Map;    % Containers for real image mapping
show_real_world_images = 0;             % Show the pixel error on real world images
encoder_std_dev_deg = 10500;            % Uncertainty on the joint angle (This is just a high random value ?)
have_true_encoder_values = 0;

% Data location #################################### <--- Important to go through each of these and modify the values
data_files.folder_path = 'data/non_overlap_static/';
data_files.measurement_type = 'train/';
data_files.real_image_path = 'real_images/'; % This is combined with folder path
data_files.transforms_file_path = strcat(data_files.folder_path,'transforms.txt');
data_files.target_file_path = strcat(data_files.folder_path,'targetParams.txt');
data_files.sensors_file_path = strcat(data_files.folder_path,'sensor_param.txt');
data_files.calibration_params_file_path = strcat(data_files.folder_path, 'minimalparam_init.txt'); % Use the initialization file for calibration
data_files.true_params_file_path = strcat(data_files.folder_path, 'minimalparam_true.txt');
data_files.use_random_pts = use_random_pts;

%% Initialize simulation object
dcc_obj = initDCC(data_files);
dcc_obj.reprojection_threshold = reprojection_threshold;
dcc_obj.data_files = data_files;
dcc_obj.optimize_theta_flag = sum(dcc_obj.optimize_theta_flag_vec)>0;
dcc_obj.add_identity_residual = add_identity_residual;
dcc_obj.bad_meas_idxs = bad_meas_idxs;
dcc_obj.real_image_mapping = real_image_mapping;
dcc_obj.show_real_world_images = show_real_world_images;
dcc_obj.use_random_points = use_random_pts;
dcc_obj.have_true_encoder_values = have_true_encoder_values;

% Get measurements
[measurement_set, dcc_obj] = loadMeasurements(measurement_vec, dcc_obj);

%%% Now we need to set up the optimization parameters %%%%
% Note that the order is from moving to static
opt_problem = setupOptimizationProblem(dcc_obj, measurement_set);

% Setup optimizer params
opt_params.gradient_norm_threshold = 1e-12;
opt_params.step_norm_threshold = 1e-12;
opt_params.max_iterations = 150;
opt_params.success = 0;

dcc_obj.caz = 0;
dcc_obj.cel = 0;
disp('Showing DCC configuration at [0 0 0] angles');
[~] = displaySimulationObject(dcc_obj, [0 0 0], opt_problem);
[caz,cel] = view;
dcc_obj.caz = caz;
dcc_obj.cel = cel;

 % Optimize for Calibration Parameters
[dcc_obj, opt_problem] = calibrateMechanism(dcc_obj, opt_problem, opt_params, measurement_set);

% Write the calibrated values to file
writeCalibratedValues(dcc_obj, opt_problem);

% Calculate the error between true and calibrated values
calcParamDiff(data_files);