function single_measurement_set = loadDataGroup(seq_num, camera_group, data_files)
%% Loads a data file a single data group and stores the poses, pixels, 3D
% points for all the cameras belonging to one measurement set

num_cameras = length(camera_group);

% For each camera
for i=1:num_cameras
    camera_name = camera_group{i}.sensor_name; 
    file_string = strcat(data_files.folder_path, data_files.measurement_type, num2str(seq_num),'_',camera_name,'.txt');
    
    % Read the measruement if the file exists
    if isfile(file_string)
        [T_CW, T_CW_cov, target_points, pixels, gimbal_angles] = loadSingleMeasurement(file_string);

        % Calculates the PnP sanity check
        if ~isempty(target_points) && ~isempty(pixels)
            [T_CW_data, ~] = solvePnPBA(target_points, pixels, camera_group{i}, T_CW);
            T_CW = T_CW_data.matrix;
            T_CW_cov = T_CW_data.cov;
        end
    else
        T_CW = [];
        T_CW_cov = [];
        target_points = [];
        pixels = [];
        gimbal_angles = [];
    end
    
    % Stores the measurements for all cameras for the particular
    % measurement
    single_measurement_set(i).T_CW = T_CW;
    single_measurement_set(i).T_CW_cov = T_CW_cov;
    single_measurement_set(i).name = camera_name;
    single_measurement_set(i).target_points = target_points;
    single_measurement_set(i).pixels = pixels;
    single_measurement_set(i).gimbal_angles = gimbal_angles;
    single_measurement_set(i).L2error.mean = -1;
    single_measurement_set(i).L2error.max = -1;
    single_measurement_set(i).L2error.min = -1;
    
    % Calculate the L2 error for reporting later
    if ~isempty(target_points) && ~isempty(pixels)
        target_pts_in_camera = applyTransform(T_CW, target_points);
        projected_pixels = camera_group{i}.project(target_pts_in_camera);
        single_measurement_set(i).L2error = getL2Error(projected_pixels, pixels);
    end
end