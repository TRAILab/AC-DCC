function cameras = loadCameraData(all_file_path)
%% Loads camera data

sensor_file_path = all_file_path.sensors_file_path;
folder_path = all_file_path.folder_path;

assert(isfile(sensor_file_path) == 1, strcat(sensor_file_path, ' does not exist'));

% Open the file
fid = fopen(sensor_file_path);

% wait for start
tline = fgetl(fid);
while(~strcmp(tline,'start:'))
    tline = fgetl(fid);
end

camera_count = 0;
tline = fgetl(fid);

% For each sensor read the data and create objects if necessary
while(~strcmp(tline,'end:'))
    camera_count = camera_count + 1;
    %cameras{camera_count} = {};
    sensor_filename = tline;
    yaml_map = readYaml(strcat(folder_path,sensor_filename));
    
    % Loads Pinhole Camera, Other cameras will need to be implemented
    cameras{camera_count} = PinholeCamera(yaml_map);
    tline = fgetl(fid);
end