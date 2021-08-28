function [measurement_set, dcc_obj] = loadMeasurements(dcc_obj)

num_measurements = dcc_obj.num_measurements;

min_pix_error_vec = zeros(num_measurements, length(dcc_obj.cameras));
avg_pix_error_vec = zeros(num_measurements, length(dcc_obj.cameras));
max_pix_error_vec = zeros(num_measurements, length(dcc_obj.cameras));

% Variables to hold measurement
measurement_set = {};
cameras = dcc_obj.cameras;
reprojection_threshold = dcc_obj.reprojection_threshold;
dcc_obj.good_meas_idxs = [];
bad_meas_counter = 0;
dcc_obj.encoder_collection = [];

% Load measurements
for meas_num = 1:num_measurements
    
    ignore_meas = 0;
    
    if ismember(meas_num, dcc_obj.bad_meas_idxs)
        bad_meas_counter = bad_meas_counter + 1;
        disp(strcat("Ignoring the measurement set ", num2str(meas_num)));
        disp("********************************");
        avg_pix_error_vec(meas_num, :) = -1;
        min_pix_error_vec(meas_num, :) = -1;
        max_pix_error_vec(meas_num, :) = -1;
        continue;
    end
    
    % Get the single measurement set
    single_measurement_set = loadDataGroup(meas_num, cameras, dcc_obj.data_files);
    
    % Group all the errors together
    temp = [single_measurement_set(:).L2error];
    avg_pix_error_vec(meas_num,:) = [temp(:).mean];
    min_pix_error_vec(meas_num,:) = [temp(:).min];
    max_pix_error_vec(meas_num,:) = [temp(:).max];
    
    % Go through each camera and determine if we should accept or reject
    for c=1:length(cameras)
        current_meas.T_CW{c} = single_measurement_set(c).T_CW;
        current_meas.T_CW_cov{c} = single_measurement_set(c).T_CW_cov;
        current_meas.target_points{c} = single_measurement_set(c).target_points;
        current_meas.pixels{c} = single_measurement_set(c).pixels;
        
        if c==1
            re = avg_pix_error_vec(meas_num, c);
            if isempty(current_meas.T_CW{c}) || re > reprojection_threshold
                disp(strcat("Ignoring the measurement set ", num2str(meas_num), " ,RE: ", num2str(re)));
                disp("********************************");
                bad_meas_counter = bad_meas_counter + 1;
                dcc_obj.bad_meas_idxs = [dcc_obj.bad_meas_idxs, meas_num];
                ignore_meas = 1;
                avg_pix_error_vec(meas_num, :) = -1;
                min_pix_error_vec(meas_num, :) = -1;
                max_pix_error_vec(meas_num, :) = -1;
                break;
            end
        else
            if ~isempty(single_measurement_set(c).T_CW)
                current_meas.T_SM{c-1} = single_measurement_set(c).T_CW/single_measurement_set(1).T_CW; % T_SW*T_WM;
                gimbal_target_points = single_measurement_set(1).target_points;
                static_target_points = single_measurement_set(c).target_points;
                static_pixels = single_measurement_set(c).pixels;
                [~, common_gimbal_idxs, common_static_idxs] = intersect(gimbal_target_points, static_target_points,'rows');
                common_target_points = gimbal_target_points(common_gimbal_idxs,:);
                current_meas.common_target_points{c-1} = common_target_points;
                current_meas.common_pixels{c-1} = static_pixels(common_static_idxs,:);
            else
                current_meas.T_SM{c-1} = [];
            end
        end
    end
    if ~ignore_meas
        disp(strcat("Adding the measurement set ",num2str(meas_num)));
        disp("==================================");
        current_meas.theta_vec = single_measurement_set(1).gimbal_angles;
        measurement_set = [measurement_set {current_meas}];  
        dcc_obj.encoder_collection = [dcc_obj.encoder_collection; current_meas.theta_vec];
    end
end

dcc_obj.bad_meas_idxs = sort(dcc_obj.bad_meas_idxs);

%% Plot average reprojection error and top K images with high average reprojection error
for i=1:length(dcc_obj.cameras)
    callFigure('Measurement Set Reproj Error');
    subplot(length(dcc_obj.cameras), 1, i);
    ape = avg_pix_error_vec(:,i);
    bar(ape);
    hold on;
    plot(xlim,[reprojection_threshold reprojection_threshold], 'r')
    xticks(1:2:num_measurements);
    temp_cam_name = string(dcc_obj.cameras{i}.sensor_name);
    temp_cam_name = join(split(temp_cam_name,'_'));
    title([strcat('Avg pix error for each measurement set for:- ',temp_cam_name),strcat(' is mean: ', num2str(mean(ape(ape>0))), ' and std: ', num2str(std(ape(ape>0))))]);
    scatter(1:size(avg_pix_error_vec,1), max_pix_error_vec(:,i), 'filled', 'k');
    scatter(1:size(avg_pix_error_vec,1), min_pix_error_vec(:,i), 'filled', 'k');
end

callFigure('Encoder Measurements');
dcc_obj.encoder_collection = rad2deg(dcc_obj.encoder_collection);
scatter3(dcc_obj.encoder_collection(:,1), dcc_obj.encoder_collection(:,2), dcc_obj.encoder_collection(:,3), 'r', 'filled');
xlabel('Pitch');
ylabel('Roll');
zlabel('Yaw');
title('Encoder angles in measurements');

opt_problem.good_meas_idxs = setdiff(1:num_measurements, dcc_obj.bad_meas_idxs);