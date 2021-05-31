function [measurement_set, simulation_object] = loadMeasurements(measurement_vec, simulation_object)

min_pix_error_vec = zeros(max(measurement_vec), length(simulation_object.cameras));
avg_pix_error_vec = zeros(max(measurement_vec), length(simulation_object.cameras));
max_pix_error_vec = zeros(max(measurement_vec), length(simulation_object.cameras));

% Variables to hold measurement
measurement_set = {};
good_measurement = 0;
cameras = simulation_object.cameras;
reprojection_threshold = simulation_object.reprojection_threshold;
simulation_object.good_meas_idxs = [];
bad_meas_counter = 0;
simulation_object.encoder_collection = [];

% Load measurements
for meas_num_temp = 1:length(measurement_vec)
    meas_num_temp
    
    % Get the measurement number
    meas_num = measurement_vec(meas_num_temp);
    
    if(~isempty(find(simulation_object.bad_meas_idxs==meas_num, 1)))
        bad_meas_counter = bad_meas_counter + 1;
        continue;
    end
    
    % Get the single measurement set
    single_measurement_set = loadDataGroup(meas_num, cameras, simulation_object.data_files);
    
    temp = [single_measurement_set(:).L2error];
    all_mean_error = temp(:).mean;
    
    if(max(all_mean_error)>reprojection_threshold)
        bad_meas_counter = bad_meas_counter + 1;
        simulation_object.bad_meas_idxs = [simulation_object.bad_meas_idxs; meas_num];
        msg = ['Data group ', num2str(meas_num), ' has avg error ', num2str(max(all_mean_error)),' larger than threshold ', num2str(reprojection_threshold), ' ,skipping...'];
        disp(msg);
        continue;
    else
        % Display which meaurement
        msg = [strcat('Loaded data group: ', num2str(meas_num)), ' with maximum average error: ', num2str(max(all_mean_error))];
        display(msg);
        good_measurement = good_measurement + 1;
        simulation_object.good_meas_idxs = [simulation_object.good_meas_idxs; meas_num];
        simulation_object.real_image_mapping(num2str(good_measurement)) = good_measurement + bad_meas_counter;
        simulation_object.encoder_collection = [simulation_object.encoder_collection; single_measurement_set(1).gimbal_angles];
    end
    
    current_meas.theta_vec = single_measurement_set(1).gimbal_angles;
    
    for c=1:length(cameras)
        current_meas.T_CW{c} = single_measurement_set(c).T_CW;
        current_meas.T_CW_cov{c} = single_measurement_set(c).T_CW_cov;
        current_meas.target_points{c} = single_measurement_set(c).target_points;
        current_meas.pixels{c} = single_measurement_set(c).pixels;
        if c<=length(cameras)-1
            current_meas.T_SM{c} = single_measurement_set(c+1).T_CW/single_measurement_set(1).T_CW; % T_SW*T_WM;
        end
    end
    
    % Add to measurement set
    measurement_set{good_measurement} = current_meas;
    
end

simulation_object.bad_meas_idxs = sort(simulation_object.bad_meas_idxs);

%% Plot average reprojection error and top K images with high average reprojection error
fig_num = 5;
for i=1:length(simulation_object.camera)
    figure(fig_num);
    subplot(length(simulation_object.camera),1,i);
    bar(avg_pix_error_vec(:,i));
    hold on;
    plot(xlim,[reprojection_threshold reprojection_threshold], 'r')
    %legend([string(simulation_object.camera_intrinsics(i).name); "reprojection threshold"]);
    xticks(1:2:length(measurement_vec));
    temp_cam_name = string(simulation_object.camera_intrinsics(i).name);
    temp_cam_name = join(split(temp_cam_name,'_'));
    title(['Avg pix error for each measurement set for ',temp_cam_name,' is mean: ', num2str(mean(avg_pix_error_vec(:,i))), ' and std: ', num2str(std(avg_pix_error_vec(:,i)))]);
    scatter(1:size(avg_pix_error_vec,1), max_pix_error_vec(:,i), 'filled', 'k');
    scatter(1:size(avg_pix_error_vec,1), min_pix_error_vec(:,i), 'filled', 'k');
    [values,indices] = maxk(avg_pix_error_vec(:,i), 9);
    if i==1 && simulation_object.show_real_world_images
        for j=1:length(values)
            figure(fig_num+i);
            subplot(3,3,j);
            imshow(strcat(simulation_object.data_files.folder_path,simulation_object.data_files.real_image_path,simulation_object.camera_intrinsics(i).name,'_',num2str(indices(j)),'.jpeg'));
            title(['Img=',num2str(indices(j)),', avg=',num2str(values(j)),', max= ', num2str(max_pix_error_vec(indices(j),i))]);
        end
    end
end

figure(7);
simulation_object.encoder_collection = rad2deg(simulation_object.encoder_collection);
scatter3(simulation_object.encoder_collection(:,1), simulation_object.encoder_collection(:,2), simulation_object.encoder_collection(:,3), 'r', 'filled');
xlabel('Pitch');
ylabel('Roll');
zlabel('Yaw');
title('Encoder angles in measurements');