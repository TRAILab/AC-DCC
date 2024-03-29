clear variables
clc
close all
fclose('all');

%% This is the main file to run to perform calibration

format long

%% Variable setup #################################### <--- Important to go through each of these and modify the values
reprojection_threshold = 5;             % Allowed reprojection threshold to decide if a measurement is good or not
axis_len = 0.4;                         % Length of the axis (for display purposes)
use_random_pts = 0;                     % Use random points in the environment (1) or a target (0) 
move_base = 0;                          % Decide if you want to move the drone
add_identity_residual = 0;              % This is the loop residual for all static cameras
num_measurements = 50;            %-----% What measurements do we want to analyze
bad_meas_idxs = [];                     % If we know any measurements are bad and dont want to include them in the optimization
show_real_world_images = 0;             % Show the pixel error on real world images
encoder_std_dev_deg = 10500;            % Uncertainty on the joint angle (This is just a high random value ?)
have_true_values = 1;
reproj_error_formulation = 0;     %-----% Whether we want the reprojection error or pose loop formulation
pixel_noise.mean = 0;                   % Pixel noise mean
pixel_noise.std_dev = 0.2;             % Pixel noise std dev (This is mainly used for the covariance in the optimization)

% Setup optimizer params #######################################
opt_params.gradient_norm_threshold = 1e-12;
opt_params.step_norm_threshold = 1e-12;
opt_params.max_iterations = 2000;
opt_params.success = 0;
opt_params.opt_type = 'LM2'; % Options: GN, LM1, LM2

% Data location #################################### <--- Important to go through each of these and modify the values
data_files.folder_path = 'data/other_experiments/four_joint/';
data_files.measurement_type = 'train/';
data_files.real_image_path = strcat(data_files.folder_path,'train_real_images/');
data_files.transforms_file_path = strcat(data_files.folder_path,'transforms.txt'); % DCC relation in the world
data_files.target_file_path = strcat(data_files.folder_path,'targetParams.txt'); % Parameters of the target if necessary
data_files.sensors_file_path = strcat(data_files.folder_path,'sensorParams.txt'); % List of all the sensor intrinsic files
data_files.calibration_params_file_path = strcat(data_files.folder_path, 'initParams.txt'); % Use the initialization file for calibration
data_files.true_params_file_path = strcat(data_files.folder_path, 'trueParams.txt'); % True DCC calibration values
data_files.optimized_params_file_path = strcat(data_files.folder_path, 'optimizedParams.txt'); % Where the output values will be written
data_files.use_random_pts = use_random_pts;

%% Initialize simulation object
dcc_obj = initDCC(data_files);
dcc_obj = loadTransformsAndTarget(data_files, dcc_obj);
dcc_obj.reprojection_threshold = reprojection_threshold;
dcc_obj.data_files = data_files;
dcc_obj.optimize_theta_flag = sum(dcc_obj.optimize_theta_flag_vec)>0;
dcc_obj.add_identity_residual = add_identity_residual;
dcc_obj.bad_meas_idxs = bad_meas_idxs;
dcc_obj.show_real_world_images = show_real_world_images;
dcc_obj.use_random_points = use_random_pts;
dcc_obj.num_measurements = num_measurements;
dcc_obj.reproj_error_formulation = reproj_error_formulation;
dcc_obj.pixel_noise = pixel_noise;

% Loads measurements
[measurement_set, dcc_obj] = loadMeasurements(dcc_obj);
%load('data/realworld1/train/dcc_obj.mat');
%load('data/realworld1/train/measurement_set.mat');
%dcc_obj.show_real_world_images = 1;
%dcc_obj.data_files.real_image_path = 'data/realworld1/real_images/';

% Setup the optimization problem
opt_problem = setupOptimizationProblem(dcc_obj, measurement_set);

opt_problem.opt_params = opt_params;

% Display the DCC object
dcc_obj.caz = 0;
dcc_obj.cel = 0;
disp('Showing DCC configuration at zero angles');
total_angles = getTotalAngles(dcc_obj.link_struct);
[~] = displaySimulationObject(dcc_obj, zeros(1,length(dcc_obj.optimize_theta_flag_vec)), opt_problem);
[caz,cel] = view;
dcc_obj.caz = caz;
dcc_obj.cel = cel;

 % Optimize for Calibration Parameters
[dcc_obj, opt_problem] = calibrateMechanism(dcc_obj, opt_problem, measurement_set);

% Write the calibrated values to file
if contains(data_files.measurement_type,'train')
    writeCalibratedValues(dcc_obj, opt_problem);
end

% Calculate the error between true, initialized and calibrated values
if have_true_values
    disp("=========================================");
    disp('Difference between true and initial values');
    calcParamDiff(data_files.true_params_file_path, data_files.calibration_params_file_path);
    disp("=========================================");
    disp('Difference between true and optimized values');
    calcParamDiff(data_files.true_params_file_path, data_files.optimized_params_file_path);
end