function single_measurement_set = loadDataGroup(seq_num, camera_group, data_files)

% Loads a data file and computes pixel error between coresponding points
% seen in both images. Also stores corresponding 3D points and pixel
% locations.

num_cameras = length(camera_group);

% For each camera
for i=1:num_cameras
    camera_name = camera_group{i}.sensor_name; 
    file_string = strcat(data_files.folder_path, data_files.measurement_type, num2str(seq_num),'_',camera_name,'.txt');
    
    assert(isfile(file_string)==1, 'The file does not exist');

    [T_CW, T_CW_cov, target_points, pixels, gimbal_angles] = loadSingleMeasurement(file_string);
    
    % Calculate the PnP sanity check
    if ~isempty(target_points) && ~isempty(pixels)
        [T_CW_data, ~] = solvePnPBA(target_points, pixels, camera_group{i}, T_CW);
        T_CW = T_CW_data.matrix;
        T_CW_cov = T_CW_data.cov;
    end
    
    single_measurement_set(i).T_CW = T_CW;
    single_measurement_set(i).T_CW_cov = T_CW_cov;
    single_measurement_set(i).name = camera_name;
    single_measurement_set(i).target_points = target_points;
    single_measurement_set(i).pixels = pixels;
    single_measurement_set(i).gimbal_angles = gimbal_angles;
    single_measurement_set(i).L2error.mean = -1;
    single_measurement_set(i).L2error.max = -1;
    single_measurement_set(i).L2error.min = -1;
    
    if ~isempty(target_points) && ~isempty(pixels)
        target_pts_in_camera = applyTransform(T_CW, target_points);
        projected_pixels = camera_group{i}.project(target_pts_in_camera);
        single_measurement_set(i).L2error = getL2Error(projected_pixels, pixels);
    end
end