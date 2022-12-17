function transforms = loadTransforms(filename)
%% This function reads the transforms for the environment setup

fid = fopen(filename);

tline = fgetl(fid);

% Read world Transform. For visualization, this is the world frame. Usually
% identity.
while(~strcmp(tline,'world:'))
    tline = fgetl(fid);
end

tline = fgetl(fid); 
for i=1:4
    temp = sscanf(tline,'%f %f %f %f');
    temp = temp';
    t(i,:) = temp;
    tline = fgetl(fid);
end
transforms.world = t;

% Reads the transformation from target frame to world frame. If we are
% using the cube environment, this this is usually identity, since we
% assume the center of the cube is the world frame.
while(~strcmp(tline,'T_WT:'))
    tline = fgetl(fid);
end

tline = fgetl(fid);
for i=1:4
    temp = sscanf(tline,'%f %f %f %f');
    temp = temp';
    t(i,:) = temp;
    tline = fgetl(fid);
end
transforms.world_T_target = t;

% Reads the transformation from base of the mechanism to the world frame.
while(~strcmp(tline,'T_WB:'))
    tline = fgetl(fid);
end

tline = fgetl(fid);
for i=1:4
    temp = sscanf(tline,'%f %f %f %f');
    temp = temp';
    t(i,:) = temp;
    tline = fgetl(fid);
end
transforms.world_T_base = t;

fclose(fid);