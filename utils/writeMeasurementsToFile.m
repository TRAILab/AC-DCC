function sim_obj = writeMeasurementsToFile(sim_obj, input_angles)

% Store Variables
world_T_target = sim_obj.transforms.world_T_target;
target_pts = sim_obj.target_pts;
folder_path = strcat(sim_obj.data_files.folder_path, sim_obj.data_files.measurement_type);
encoder_angles_rad = input_angles;
num_collected_measurements = sim_obj.num_collected_measurements;
optimize_theta_flag_vec = sim_obj.optimize_theta_flag_vec;
successful_measurement_collection_flag = zeros(size(input_angles,1),1);
noisy_encoder_angles_rad_list = zeros(size(input_angles));
bad_measurement_num = [];
all_measurement_pixel_error_mean = zeros(size(encoder_angles_rad,1),length(sim_obj.cameras));

opt_problem = setupOptimizationProblem(sim_obj, []);

for i=1:size(encoder_angles_rad,1)
    
    fprintf(['\n',num2str(i),' ==================================\n']);
    
    % We want to add noise to the encoder angle before we move to get the
    % measurement. Eg. If we have to collect a measurement from 20 deg, the
    % simulator should move to a slightly different angle based on the
    % encoder std dev and get the measurement. However it should report the
    % true encoder angle in the measurement (here 20 deg).
    % Add noise to encoder angles
    noisy_encoder_angles_rad = addNoise(encoder_angles_rad(i,:), 'enc', sim_obj.encoder_noise, optimize_theta_flag_vec);
    
    % Randomly move the base
%     if simulation_object.move_base
%         disp('Moving base');
%         rand_trans = get_3d_pts_in_range(-0.05, 0.05, 3);
%         rand_rot = deg2rad(get_3d_pts_in_range(-5, 5, 3));
%         oldbase_T_newbase = Transformation([rand_rot rand_trans]).matrix;
%         simulation_object.transforms.w_T_base = simulation_object.transforms.w_T_base*oldbase_T_newbase;
%     end
    
    % Display the object. Returns transforms for use at other places
    T_WC_list = displaySimulationObject(sim_obj, noisy_encoder_angles_rad, opt_problem);
    
    cam_T_target_list = {};
    cam_success_list = [];
    L2_error_cam_list = {};
    target_pts_seen_in_cam_list = {};
    noisy_cam_pix_on_plane_list = {};
    
    % Go through all cameras
    for c=1:length(sim_obj.cameras)
        
        % Get the transformation from camera to world
        w_T_cam = T_WC_list{c};
        
        % Get target pts in static frame. INV(A)*b = A\b
        target_pts_in_cam_frame = applyTransform(w_T_cam\world_T_target, target_pts);

        % Project camera points on image plane
        all_cam_pixels = sim_obj.cameras{c}.project(target_pts_in_cam_frame);

        % Add noise to pixels
        all_noisy_cam_pixels = addNoise(all_cam_pixels, 'pix', sim_obj.pixel_noise, []);

        % Get pixels that fall on the plane and corresponding indices of 3D points
        cam_indices_on_plane = sim_obj.cameras{c}.getIndicesOnImage(all_noisy_cam_pixels, target_pts_in_cam_frame);

        % Get noisy pixels on the image plane
        noisy_cam_pix_on_plane = all_noisy_cam_pixels(cam_indices_on_plane,:);

        % Get the Target points for which the pixels are on the plane
        target_pts_seen_in_cam = target_pts(cam_indices_on_plane,:);
        
        if size(target_pts_seen_in_cam, 1)>4
            disp(strcat('Solving PnP for Cam: ', sim_obj.cameras{c}.sensor_name));
            cam_T_target_orig = w_T_cam\world_T_target;
            noisy_cam_T_target = addNoise(cam_T_target_orig, 'transformation', sim_obj.transformation_noise, []);
            [cam_T_target, L2_error] = solvePnPBA(target_pts_seen_in_cam, noisy_cam_pix_on_plane, sim_obj.cameras{c}, noisy_cam_T_target);
        end
        
        cam_T_target_list(c) = {cam_T_target};
        cam_success_list(c) = L2error.mean < sim_obj.reprojection_threshold;
        L2_error_cam_list{c} = L2_error;
        target_pts_seen_in_cam_list(c) = {target_pts_seen_in_cam};
        noisy_cam_pix_on_plane_list(c) = {noisy_cam_pix_on_plane};
        
        disp(strcat('Average error on: ', sim_obj.cameras{c}.sensor_name,'=',num2str(L2_error.mean)));
        disp(' ');
        all_measurement_pixel_error_mean(i,c) = L2_error.mean;
    end
    
    % Write measurement to file.
    if sum(cam_success_list) == length(cam_success_list)
        successful_measurement_collection_flag(i) = 1;
        num_collected_measurements = num_collected_measurements + 1;
        noisy_encoder_angles_rad_list(num_collected_measurements, :) = noisy_encoder_angles_rad;
        
        for c=1:length(sim_obj.cameras)
            pts_pix = [target_pts_seen_in_cam_list{c} noisy_cam_pix_on_plane_list{c}];
            writeToFile(num_collected_measurements, sim_obj.cameras{c}.sensor_name, folder_path, pts_pix, cam_T_target_list{c}, encoder_angles_rad(i,:));
            display(strcat(['Wrote measurement for encoder set in deg:', num2str(rad2deg(noisy_encoder_angles_rad)), ' with ', num2str(size(target_pts_seen_in_cam_list{c},1)), ' ',sim_obj.cameras{c}.sensor_name,' points.']));
        end
        
        sim_obj.num_collected_measurements = sim_obj.num_collected_measurements + 1;
        
        disp('---------------------------------------------------------------');
    else
        disp('Could not write to file, reprojection error was high');
        display(strcat(['Average Static1 Error ',num2str(avg_err_static1)]));
        display(strcat(['Average Static2 Error ',num2str(avg_err_static2)]));
        display(strcat(['Average Gimbal Error ',num2str(avg_err_gimbal)]));
        bad_measurement_num = [bad_measurement_num 1];
    end

end

fprintf(strcat('Collected \t',num2str(num_collected_measurements),'/',num2str(size(encoder_angles_rad,1)),' measurements'));
sim_obj.bad_measurement_num = bad_measurement_num;

%% Write noisy encoder angles to a new file
complete_path = strcat(folder_path, 'true_encoder_angles.txt');
fileID = fopen(complete_path,'wt');
fprintf(fileID,'True encoder values\n');
writematrix(noisy_encoder_angles_rad_list, complete_path);
fclose(fileID);

%% Plot the figure of the mean reprojection error for each measurment and camera
for i=1:length(sim_obj.cameras)
    callFigure('Measurement Set Reproj Error');
    subplot(length(sim_obj.cameras), 1, i);
    bar(all_measurement_pixel_error_mean(:,i));
    hold on;
    xticks(1:2:length(all_measurement_pixel_error_mean));
    temp_cam_name = string(sim_obj.cameras{i}.sensor_name);
    temp_cam_name = join(split(temp_cam_name,'_'));
    title([strcat('Avg pix error for each measurement set for:- ',temp_cam_name),strcat(' is mean: ', num2str(mean(all_measurement_pixel_error_mean(:,i))), ' and std: ', num2str(std(all_measurement_pixel_error_mean(:,i))))]);
end