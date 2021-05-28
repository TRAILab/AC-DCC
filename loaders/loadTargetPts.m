function target_pts_CB = loadTargetPts(filepath)

% Chessboard parameters
% sqSize = 0.10; % chessboard square size in meters
% width = 9; % number of internal corners in width and height
% height = 7;

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

% Create Target coordinates in target frame {CB}
target_pts_CB = [];
for w=0:width-1
    for h=0:height-1
        target_pts_CB = [target_pts_CB;
                        sqSize*w sqSize*h 0];
    end
end

fclose(fid);