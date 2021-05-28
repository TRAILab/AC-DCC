function transforms = loadTransforms(filename)

% This function reads the transforms for the environment setup

fid = fopen(filename);

tline = fgetl(fid); % First line description

% Read world Transform
while(~strcmp(tline,'world:'))
    tline = fgetl(fid);
end

tline = fgetl(fid); % First set of parameters
for i=1:4
    temp = sscanf(tline,'%f %f %f %f');
    temp = temp';
    t(i,:) = temp;
    tline = fgetl(fid);
end
transforms.world = t;

% Read target to world transform
while(~strcmp(tline,'world_T_target:'))
    tline = fgetl(fid);
end

tline = fgetl(fid); % First set of parameters
for i=1:4
    temp = sscanf(tline,'%f %f %f %f');
    temp = temp';
    t(i,:) = temp;
    tline = fgetl(fid);
end
transforms.world_T_target = t;

% Read base of mechanism to world transform
while(~strcmp(tline,'world_T_base:'))
    tline = fgetl(fid);
end

tline = fgetl(fid); % First set of parameters
for i=1:4
    temp = sscanf(tline,'%f %f %f %f');
    temp = temp';
    t(i,:) = temp;
    tline = fgetl(fid);
end
transforms.world_T_base = t;

fclose(fid);