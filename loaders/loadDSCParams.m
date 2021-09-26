function params_struct = loadDSCParams(filename)

%% Description:
% This file reads the parameter file and returns the DCC parameters

fid = fopen(filename); % get the commented line

% wait for start
tline = fgetl(fid);
while(~strcmp(tline,'start:'))
    tline = fgetl(fid);
end

% Read the moving camera to previous joint transformation
tline = fgetl(fid);
temp_str = split(tline,',');
if strcmp(temp_str{1}, 'mdh')
    camScanString = '%s %f %f %f %f %f %f %f %d %d %d %d %d %d %f %f';
    C = textscan(tline, camScanString, 1, 'delimiter', ',');
    params_struct(1).params = cell2mat(C(2:7));
    params_struct(1).offset = cell2mat(C(8));
    params_struct(1).opt = cell2mat(C(9:14));
    params_struct(1).bounds = cell2mat(C(15:16));
    params_struct(1).type = 'mdh';
else
    camScanString = '%s %f %f %f %f %f %f %d %d %d %d %d %d';
    C = textscan(tline, camScanString, 1, 'delimiter', ',');
    params_struct(1).params = cell2mat(C(2:7));
    params_struct(1).opt = cell2mat(C(8:end));
    params_struct(1).type = '6dof';
end


% Get number of DH links
tline = fgetl(fid);
DHLinkCount = sscanf( tline, '%d\n' );

% Read the DH parameters and angle bounds
DHScanString = '%s %f %f %f %f %f %d %d %d %d %f %f';

% Get DH parameters
for i=1:DHLinkCount
    C = textscan( fid, DHScanString, 1, 'delimiter', ',' );
    params_struct(i+1).params = cell2mat(C(2:5));
    params_struct(i+1).offset = cell2mat(C(6));
    params_struct(i+1).opt = cell2mat(C(7:10));
    params_struct(i+1).bounds = cell2mat(C(11:12));
    params_struct(i+1).type = 'dh';
end

% Get 6 dof from base to static cameras
camScanString = '%s %f %f %f %f %f %f %d %d %d %d %d %d';
camScanString4dof = '%s %f %f %f %f %d %d %d %d';
count_static_cam = 1;
while(~strcmp(tline,'end:'))
    C = textscan( fid, camScanString, 1, 'delimiter', ',' );
    if strcmp(cell2mat(C{1}),'end:')
        break
    end
    
    if strcmp(cell2mat(C{1}),'4dof')
        params_struct(1+DHLinkCount+count_static_cam).params = cell2mat(C(2:5));
        params_struct(1+DHLinkCount+count_static_cam).opt(1) = cell2mat(C(6));
        params_struct(1+DHLinkCount+count_static_cam).opt(2) = cell2mat(C(7));
        params_struct(1+DHLinkCount+count_static_cam).opt(3) = cell2mat(C(8));
        params_struct(1+DHLinkCount+count_static_cam).opt(4) = cell2mat(C(9));
        params_struct(1+DHLinkCount+count_static_cam).type = '4dof';
    end
    
    if strcmp(cell2mat(C{1}),'6dof')
        params_struct(1+DHLinkCount+count_static_cam).params = cell2mat(C(2:7));
        params_struct(1+DHLinkCount+count_static_cam).opt = cell2mat(C(8:end));
        params_struct(1+DHLinkCount+count_static_cam).type = '6dof';
    end
    
    count_static_cam = count_static_cam + 1;
end

fclose(fid);