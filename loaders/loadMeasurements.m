function [measurement_set, dcc_obj] = loadMeasurements(measurement_vec, dcc_obj)

min_pix_error_vec = zeros(max(measurement_vec), length(dcc_obj.cameras));
avg_pix_error_vec = zeros(max(measurement_vec), length(dcc_obj.cameras));
max_pix_error_vec = zeros(max(measurement_vec), length(dcc_obj.cameras));

% Variables to hold measurement
measurement_set = {};
good_measurement = 0;
cameras = dcc_obj.cameras;
reprojection_threshold = dcc_obj.reprojection_threshold;
dcc_obj.good_meas_idxs = [];
bad_meas_counter = 0;
dcc_obj.encoder_collection = [];

% Load measurements
for meas_num_temp = 1:length(measurement_vec)
    meas_num_temp
    
    % Get the measurement number
    meas_num = measurement_vec(meas_num_temp);
    
    if(~isempty(find(dcc_obj.bad_meas_idxs==meas_num, 1)))
        bad_meas_counter = bad_meas_counter + 1;
        continue;
    end
    
    % Get the single measurement set
    single_measurement_set = loadDataGroup(meas_num, cameras, dcc_obj.data_files);
    
    temp = [single_measurement_set(:).L2error];
    single_set_mean_error = [temp(:).mean];
    avg_pix_error_vec(meas_num_temp,:) = [temp(:).mean];
    min_pix_error_vec(meas_num_temp,:) = [temp(:).min];
    max_pix_error_vec(meas_num_temp,:) = [temp(:).max];
    
    if(max(single_set_mean_error)>reprojection_threshold)
        bad_meas_counter = bad_meas_counter + 1;
        dcc_obj.bad_meas_idxs = [dcc_obj.bad_meas_idxs; meas_num];
        msg = ['Data group ', num2str(meas_num), ' has avg error ', num2str(max(single_set_mean_error)),' larger than threshold ', num2str(reprojection_threshold), ' ,skipping...'];
        disp(msg);
        continue;
    else
        % Display which meaurement
        msg = [strcat('Loaded data group: ', num2str(meas_num)), ' with maximum average error: ', num2str(max(single_set_mean_error))];
        display(msg);
        good_measurement = good_measurement + 1;
        dcc_obj.good_meas_idxs = [dcc_obj.good_meas_idxs; meas_num];
        dcc_obj.real_image_mapping(num2str(good_measurement)) = good_measurement + bad_meas_counter;
        dcc_obj.encoder_collection = [dcc_obj.encoder_collection; single_measurement_set(1).gimbal_angles];
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

dcc_obj.bad_meas_idxs = sort(dcc_obj.bad_meas_idxs);

%% Plot average reprojection error and top K images with high average reprojection error
for i=1:length(dcc_obj.cameras)
    callFigure('Measurement Set Reproj Error');
    subplot(length(dcc_obj.cameras), 1, i);
    bar(avg_pix_error_vec(:,i));
    hold on;
    plot(xlim,[reprojection_threshold reprojection_threshold], 'r')
    xticks(1:2:length(measurement_vec));
    temp_cam_name = string(dcc_obj.cameras{i}.sensor_name);
    temp_cam_name = join(split(temp_cam_name,'_'));
    title([strcat('Avg pix error for each measurement set for:- ',temp_cam_name),strcat(' is mean: ', num2str(mean(avg_pix_error_vec(:,i))), ' and std: ', num2str(std(avg_pix_error_vec(:,i))))]);
    scatter(1:size(avg_pix_error_vec,1), max_pix_error_vec(:,i), 'filled', 'k');
    scatter(1:size(avg_pix_error_vec,1), min_pix_error_vec(:,i), 'filled', 'k');
    [values,indices] = maxk(avg_pix_error_vec(:,i), 9);
    if i==1 && dcc_obj.show_real_world_images
        for j=1:length(values)
            figure(fig_num+i);
            subplot(3,3,j);
            imshow(strcat(dcc_obj.data_files.folder_path,dcc_obj.data_files.real_image_path,dcc_obj.camera_intrinsics(i).name,'_',num2str(indices(j)),'.jpeg'));
            title(['Img=',num2str(indices(j)),', avg=',num2str(values(j)),', max= ', num2str(max_pix_error_vec(indices(j),i))]);
        end
    end
end

callFigure('Encoder Measurements');
dcc_obj.encoder_collection = rad2deg(dcc_obj.encoder_collection);
scatter3(dcc_obj.encoder_collection(:,1), dcc_obj.encoder_collection(:,2), dcc_obj.encoder_collection(:,3), 'r', 'filled');
xlabel('Pitch');
ylabel('Roll');
zlabel('Yaw');
title('Encoder angles in measurements');