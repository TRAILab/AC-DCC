function sim_obj = writeMeasurementsToFile(sim_obj, input_angles)

%% Description:
% This function is used to generate simulated measurements for calibration
% and writes it to a file

% Store Variables
T_WT = sim_obj.transforms.world_T_target;
target_pts = sim_obj.target_pts;
folder_path = strcat(sim_obj.data_files.folder_path, sim_obj.data_files.measurement_type);
encoder_angles_rad = input_angles;
optimize_theta_flag_vec = sim_obj.optimize_theta_flag_vec;
noisy_encoder_angles_rad_list = zeros(size(input_angles));
all_measurement_pixel_error_mean = zeros(size(encoder_angles_rad,1),length(sim_obj.cameras));

opt_problem = setupOptimizationProblem(sim_obj, []);

for meas_num=1:size(encoder_angles_rad,1)
    
    fprintf(['\n',num2str(meas_num),' ==================================\n']);
    
    % We want to add noise to the encoder angle before we move to get the
    % measurement. Eg. If we have to collect a measurement from 20 deg, the
    % simulator should move to a slightly different angle based on the
    % encoder std dev and get the measurement. However it should report the
    % true encoder angle in the measurement (here 20 deg).
    % Add noise to encoder angles
    noisy_encoder_angles_rad = addNoise(encoder_angles_rad(meas_num,:), 'enc', sim_obj.encoder_noise, optimize_theta_flag_vec);
    
    % Randomly move the base
    if sim_obj.move_base
        disp('Moving base');
        rand_trans = getValuesInRange(-0.05, 0.05, [1 3]);
        rand_rot = deg2rad(getValuesInRange(-5, 5, [1 3]));
        T_oldbase_newbase = Transformation([rand_rot rand_trans]).matrix;
        sim_obj.transforms.w_T_base = sim_obj.transforms.w_T_base*T_oldbase_newbase;
    end
    
    % Display the object. Returns transforms for use at other places
    T_WC_list = displaySimulationObject(sim_obj, noisy_encoder_angles_rad, opt_problem);

    % Go through all cameras
    for c=1:length(sim_obj.cameras)
        
        % Get the transformation from camera to world
        T_WC = T_WC_list{c};
        
        % Transform points from target to camera via world
        target_pts_in_cam_frame = applyTransform(T_WC\T_WT, target_pts);

        % Project camera points on image plane
        all_cam_pixels = sim_obj.cameras{c}.project(target_pts_in_cam_frame);

        % Add noise to pixels
        all_noisy_cam_pixels = addNoise(all_cam_pixels, 'pix', sim_obj.pixel_noise, []);

        % Get pixels that fall on the plane and corresponding indices of 3D points
        cam_indices_on_plane = sim_obj.cameras{c}.getIndicesOnImage(all_noisy_cam_pixels, target_pts_in_cam_frame);

        % Get noisy pixels on the image plane
        noisy_cam_pix_on_plane = all_noisy_cam_pixels(cam_indices_on_plane, :);

        % Get the Target points in the target frame, for which the pixels are on the plane
        target_pts_seen_in_cam = target_pts(cam_indices_on_plane, :);
        
        if size(target_pts_seen_in_cam, 1)>4
            disp(strcat("Solving PnP for Cam: ", sim_obj.cameras{c}.sensor_name));
            cam_T_target_orig = T_WC\T_WT;
            noisy_cam_T_target = addNoise(cam_T_target_orig, 'transformation', sim_obj.transformation_noise, []);
            [T_CW, L2_error] = solvePnPBA(target_pts_seen_in_cam, noisy_cam_pix_on_plane, sim_obj.cameras{c}, noisy_cam_T_target);
        else
            L2_error.mean = -2;
        end
        
        disp(strcat("Average error on: ", sim_obj.cameras{c}.sensor_name, "=", num2str(L2_error.mean)));
        disp(' ');
        all_measurement_pixel_error_mean(meas_num,c) = L2_error.mean;
    
        % Write measurement to file.
        if L2_error.mean < sim_obj.reprojection_threshold && L2_error.mean>-1
            noisy_encoder_angles_rad_list(meas_num, :) = noisy_encoder_angles_rad;
            pts_pix = [target_pts_seen_in_cam noisy_cam_pix_on_plane];
            writeToFile(sim_obj.last_index + meas_num, sim_obj.cameras{c}.sensor_name, folder_path, pts_pix, T_CW, encoder_angles_rad(meas_num,:));
            display(strcat("Wrote measurement for encoder set in deg: ", num2str(rad2deg(noisy_encoder_angles_rad)), " with ", num2str(size(target_pts_seen_in_cam,1)), " ",sim_obj.cameras{c}.sensor_name," points."));
        else
            display(strcat("Could not write to file: ", sim_obj.cameras{c}.sensor_name, " with error ", num2str(L2_error.mean)));
        end
    end
    disp('---------------------------------------------------------------');
end

%% Write noisy encoder angles to a new file
complete_path = strcat(folder_path, 'true_encoder_angles.txt');
fileID = fopen(complete_path,'wt');
fprintf(fileID,'True encoder values\n');
writematrix(noisy_encoder_angles_rad_list, complete_path);
fclose(fileID);

%% Plot the figure of the mean reprojection error for each measurment and camera
for c=1:length(sim_obj.cameras)
    callFigure('Measurement Set Reproj Error');
    subplot(length(sim_obj.cameras), 1, c);
    bar(all_measurement_pixel_error_mean(:,c));
    hold on;
    xticks(1:2:length(all_measurement_pixel_error_mean));
    temp_cam_name = string(sim_obj.cameras{c}.sensor_name);
    temp_cam_name = join(split(temp_cam_name,'_'));
    title([strcat('Avg pix error for each measurement set for:- ',temp_cam_name),strcat(' is mean: ', num2str(mean(all_measurement_pixel_error_mean(:,c))), ' and std: ', num2str(std(all_measurement_pixel_error_mean(:,c))))]);
end