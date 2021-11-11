clear all
clc
close all

% This file is used to generate different environments

%% Cube

hs = 2; % half side size
sep = 0.25;
pts_range = -hs:sep:hs;
pts_range = pts_range';
const_val = hs*ones(length(pts_range),1);

bottom_back_left = [-hs hs -hs];
bottom_back_right = [-hs -hs -hs];
bottom_front_left = [hs hs -hs];
bottom_front_right = [hs -hs -hs];

top_back_left = [-hs hs hs];
top_back_right = [-hs -hs hs];
top_front_left = [hs hs hs];
top_front_right = [hs -hs hs];

target_pts = [];

% Front and back
for y=-hs:sep:hs
    for z=-hs:sep:hs
        target_pts = [target_pts; hs y z; -hs y z];
    end
end

% Left and Right
for x=-hs:sep:hs
    for z=-hs:sep:hs
        target_pts = [target_pts; x hs z; x -hs z];
    end
end

% Top and Bottom
for x=-hs:sep:hs
    for y=-hs:sep:hs
        target_pts = [target_pts; x y hs; x y -hs];
    end
end

target_pts = unique(target_pts,'rows');
noise = getValuesInRange(-0.20,0.20,size(target_pts));
target_pts = target_pts + noise;

scatter3(target_pts(:,1),target_pts(:,2), target_pts(:,3),'filled','b','MarkerFaceAlpha', 0.5);
hold on;
plotAxis(eye(4), 0.2, 'r', 'W');
temp = [bottom_back_left; bottom_back_right];
line_colour = 'k';
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [bottom_back_left; bottom_front_left];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [bottom_back_right; bottom_front_right];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [bottom_front_left; bottom_front_right];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);

temp = [top_back_left; top_back_right];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [top_back_left; top_front_left];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [top_back_right; top_front_right];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [top_front_left; top_front_right];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);

temp = [bottom_back_left; top_back_left];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [bottom_back_right; top_back_right];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [bottom_front_right; top_front_right];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [bottom_front_left; top_front_left];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);

xlabel('X co-ordinate (m)');
ylabel('Y co-ordinate (m)');
zlabel('Z co-ordinate (m)');
title('Cube Environment');

%% Target
figure();
data_files.folder_path = 'data/ijrr/multicamera_test/';
data_files.sensors_file_path = strcat(data_files.folder_path,'sensorParams.txt');
data_files.transforms_file_path = strcat(data_files.folder_path,'transforms.txt');
data_files.target_file_path = strcat(data_files.folder_path,'targetParams.txt');
data_files.calibration_params_file_path = strcat(data_files.folder_path,'trueParams.txt');
data_files.use_random_pts = 0;

% Initialize simulation object
dsc_obj = initDCC(data_files);
dsc_obj = loadTransformsAndTarget(data_files, dsc_obj);
plotAxis(dsc_obj.transforms.world, 0.2, 'r', 'T^w');
% Plot target
target_pts_in_world = dsc_obj.target_pts_world;
hold on;
scatter3(target_pts_in_world(:, 1), target_pts_in_world(:, 2), target_pts_in_world(:, 3), 'filled', 'b','MarkerFaceAlpha', 0.5);
plotAxis(dsc_obj.transforms.world_T_target, 0.2, 'k', 'T^t');
xlabel('X co-ordinate (m)');
ylabel('Y co-ordinate (m)');
zlabel('Z co-ordinate (m)');
hold on;
line_colour = 'k';
temp = [target_pts_in_world(1,:); target_pts_in_world(10,:)];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [target_pts_in_world(10,:); target_pts_in_world(150,:)];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [target_pts_in_world(150,:); target_pts_in_world(141,:)];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
temp = [target_pts_in_world(1,:); target_pts_in_world(141,:)];
plot3(temp(:,1),temp(:,2),temp(:,3),line_colour,'linewidth',3);
xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
title('Target Environment');