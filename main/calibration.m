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
measurement_vec = 1:10;                 % What measurements do we want to analyze
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
have_true_encoder_values = 1;

% Data location #################################### <--- Important to go through each of these and modify the values
data_files.folder_path = 'data/test_5_cam/';
data_files.measurement_type = 'train/';
data_files.real_image_path = 'real_images/'; % This is combined with folder path
data_files.sensors_file_path = strcat(data_files.folder_path,'sensor_param.txt');
data_files.transforms_file_path = strcat(data_files.folder_path,'transforms.txt');
data_files.target_file_path = strcat(data_files.folder_path,'targetParams.txt');
data_files.calibration_params_file_path = strcat(data_files.folder_path,'overparam_true.txt');
data_files.true_encoder_angles_path = strcat(data_files.folder_path,data_files.measurement_type, 'true_encoder_angles.txt');
true_params_path = strcat(data_files.folder_path, 'trueParams.txt');
data_files.use_random_pts = use_random_pts;

%% Initialize simulation object
simulation_object = initDCC(data_files);
simulation_object.reprojection_threshold = reprojection_threshold;
simulation_object.data_files = data_files;
simulation_object.eval_flag = eval_flag;
simulation_object.optimize_scale_offset = optimize_scale_offset;
simulation_object.optimize_theta_flag = sum(optimize_theta_flag_vec)>0;
simulation_object.optimize_theta_flag_vec = optimize_theta_flag_vec;
simulation_object.multicam = multicam;
simulation_object.add_identity_residual = add_identity_residual;
simulation_object.pixel_error_formulation = pixel_error_formulation;
simulation_object.bad_meas_idxs = bad_meas_idxs;
simulation_object.real_image_mapping = real_image_mapping;
simulation_object.show_real_world_images = show_real_world_images;
simulation_object.add_prior_joint_angles = sum(prior_joint_angles_vec)>0;
simulation_object.prior_joint_angles_vec = prior_joint_angles_vec;
simulation_object.encoder_std_dev_deg = encoder_std_dev_deg;
simulation_object.encoder_std_dev_rad = deg2rad(encoder_std_dev_deg);
simulation_object.have_true_values = have_true_values;
simulation_object.show_residual_values = show_residual_values;
simulation_object.have_true_encoder_values = have_true_encoder_values;
simulation_object.use_random_points = use_random_pts;

% If we have the true encoder angles
if simulation_object.have_true_encoder_values
    simulation_object.true_encoder_angles = readmatrix(data_files.true_encoder_angles_path);
end

% Get measurements
[measurement_set, simulation_object] = loadMeasurements(measurement_vec, simulation_object);

%%% Now we need to set up the optimization parameters %%%%
% Note that the order is from moving to static
[opt_problem, ~, ~, ~] = setup_optimization_problem(simulation_object, measurement_set);

% Setup optimizer params
opt_params.gradient_norm_threshold = 1e-12;
opt_params.step_norm_threshold = 1e-12;
opt_params.max_iterations = 150;
opt_params.success = 0;

% Show simulation object
% figure(5);
% clf;
simulation_object.caz = 0;
simulation_object.cel = 0;
disp('Showing DCC configuration at [0 0 0] angles');
[~] = display_simulation_object(simulation_object, [0 0 0], opt_problem);
%pause(2);
[caz,cel] = view;
simulation_object.caz = caz;
simulation_object.cel = cel;

 % Optimize for Calibration Parameters
[simulation_object, opt_problem] = calibrate_mechanism(simulation_object, opt_problem, opt_params, measurement_set);

% Write the calibrated values to file
write_calibrated_values_to_file(simulation_object, data_files, opt_problem);

%% Get difference between true and optimized encoder angles
if simulation_object.optimize_theta_flag && have_true_encoder_values
    indxs = find(optimize_theta_flag_vec==1);
    true_encoder_angles_deg = rad2deg(readmatrix(data_files.true_encoder_angles_path));
    true_encoder_angles_deg_sep = true_encoder_angles_deg(good_meas_idxs,indxs);
    num_dh_links = length(simulation_object.link_struct)-length(simulation_object.camera_intrinsics);
    num_non_theta_params = length(opt_problem.parameter_container.parameter_list)-(length(measurement_set)*sum(optimize_theta_flag_vec));
    temp_angle_holder = [];
    for i=num_non_theta_params+1:length(opt_problem.parameter_container.parameter_list)
        temp_angle_holder = [temp_angle_holder; opt_problem.parameter_container.parameter_list{i}.parameter.value];
    end
    %temp_angle_holder = reshape(temp_angle_holder, [num_dh_links length(measurement_set)]);
    temp_angle_holder = reshape(temp_angle_holder,sum(optimize_theta_flag_vec),length(measurement_set))';
    temp_angle_holder_deg = rad2deg(temp_angle_holder);
    difference_angles = true_encoder_angles_deg_sep - temp_angle_holder_deg;
end

%% Get the true parameters
[opt_params_true, link_struct_true] = load_link_parameters(true_params_path);
true_e_m = Transformation(link_struct_true(1).default_values);
true_s_b_group{1} = Transformation(link_struct_true(1+simulation_object.num_DH_links+1).default_values);
if length(link_struct_true)>5
    true_s_b_group{2} = Transformation(link_struct_true(1+simulation_object.num_DH_links+2).default_values);
end

% Display true vs optimized values
num_params = length(opt_problem.parameter_container.parameter_list)-(length(measurement_set)*sum(optimize_theta_flag_vec));
opt_params_counter = 1;
static_camera_counter = 1;
for i=1:num_params
    if i==1
        temp = opt_problem.parameter_container.parameter_list{i}.parameter.matrix/true_e_m.matrix;
        v = tran2vec2(temp)';
        v = [rad2deg(v(1:3)) v(4:6)]
        idx_map = simulation_object.link_struct(i).index_map;
        opt_params_counter = opt_params_counter + length(idx_map(idx_map~=-1));
    elseif i>num_params-length(simulation_object.camera_intrinsics)+1 % Both static cameras
        temp = opt_problem.parameter_container.parameter_list{i}.parameter.matrix/true_s_b_group{static_camera_counter}.matrix;
        v = tran2vec2(temp)';
        v = [rad2deg(v(1:3)) v(4:6)]
        %idx_map = simulation_object.link_struct(i).index_map;
        %opt_params_counter = opt_params_counter + length(idx_map(idx_map~=-1));
        static_camera_counter = static_camera_counter+1;
    else
        opt_problem.parameter_container.parameter_list{i}.parameter.value - opt_params_true(opt_params_counter)
        opt_params_counter = opt_params_counter + 1;
    end
end

% %% Get the reprojection error
% if simulation_object.eval_flag
%     simulation_object.data_files.folder_path = data_files.evaluation_folder_path;
%     [measurement_set_validation, simulation_object] = load_measurements(eval_vec, simulation_object);
% 
%     for m=1:length(measurement_set_validation)
%         measurement_struct = measurement_set_validation{m};
%         [~, transform_chain] = movingToStaticChain(opt_problem.parameter_container, measurement_struct.theta_vec, simulation_object);
%         w_T_m = simulation_object.transforms.w_T_base*transform_chain{4}*transform_chain{3}*transform_chain{2}*transform_chain{1};
%         target_pts_in_m = applyTransform(inv(w_T_m),measurement_struct.raw_camera_points{1});
%         projected_pixels = general_camera_projection(simulation_object.camera{1}.intrinsics,target_pts_in_m);
%         error_matrix = projected_pixels-measurement_struct.raw_pixel_measurements{1};
%         error_vector = sqrt(sum(error_matrix.^2,2));
%         avg_err(m) = mean(error_vector);
%     end
%     mean(avg_err)
% end