function target_pts_CB = loadTargetPts(filepath)
%% This file loads a checkerboard target based on parameters referenced in
% the file.

fid = fopen(filepath);

tline = fgetl(fid); % First line description

tline = fgetl(fid);
tline = fgetl(fid);
temp = sscanf(tline,'%f');
sqSize = temp;

tline = fgetl(fid);
tline = fgetl(fid);
temp = sscanf(tline,'%f');
width = temp;

tline = fgetl(fid);
tline = fgetl(fid);
temp = sscanf(tline,'%f');
height = temp;

% Creates target points in target frame {CB: Checkerboard}
target_pts_CB = [];
for w=0:width-1
    for h=0:height-1
        target_pts_CB = [target_pts_CB;
                        sqSize*w sqSize*h 0];
    end
end

fclose(fid);