function writeToFile(measurement_num, camera_name, folder_path, target_pts_pix, camera_T_target_struct, raw_encoder_angles)

%% Description
% This function writes the different values to a text file for the
% calibration

if ~exist(folder_path, 'dir')
    mkdir(folder_path);
end

complete_path = strcat(folder_path,num2str(measurement_num),'_',camera_name,'.txt');

fileID = fopen(complete_path,'wt');

% Write transform
fprintf(fileID,'T_CW:\r\n');
for i=1:4
    fprintf(fileID,'%.10f %.10f %.10f %.10f\n', camera_T_target_struct.matrix(i,:));
end

fprintf(fileID,'\n');

fprintf(fileID,'T_CW_cov:\r\n');
for i=1:6
    fprintf(fileID,'%.10f %.10f %.10f %.10f %.10f %.10f\n', camera_T_target_struct.cov(i,:));
end

fprintf(fileID,'\n');

% Write Gridpoints
fprintf(fileID,'target_points_pix:\n');
for i=1:size(target_pts_pix,1)
    fprintf(fileID, '%.10f %.10f %.10f %.10f %.10f\n',target_pts_pix(i,:));
end

fprintf(fileID,'\n');

% Write Angles
fprintf(fileID,'gimbalangles:\n');
if(length(raw_encoder_angles)==1)
    fprintf(fileID,'%.10f\n',raw_encoder_angles(1,:));
elseif(length(raw_encoder_angles)==2)
    fprintf(fileID,'%.10f %.10f\n',raw_encoder_angles(1,:));
elseif(length(raw_encoder_angles)==3)
    fprintf(fileID,'%.10f %.10f %.10f\n',raw_encoder_angles(1,:));
end

fprintf(fileID,'\n');

fprintf(fileID,'end:');
fclose(fileID);