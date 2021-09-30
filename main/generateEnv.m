clear all
clc
close all

% This file is used to generate different environments

% Cube
hs = 2; % half side size
sep = 0.25;
pts_range = -hs:sep:hs;
pts_range = pts_range';
const_val = hs*ones(length(pts_range),1);

bottom_back_left = [-hs -hs -hs];
bottom_back_right = [-hs -hs hs];
bottom_front_left = [-hs hs -hs];
bottom_front_right = [-hs hs hs];

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

scatter3(target_pts(:,1),target_pts(:,2), target_pts(:,3),'filled','k');
hold on;
temp = [bottom_back_left; bottom_back_right];
plot3(temp(:,1),temp(:,2),temp(:,3),'r','linewidth',3);
temp = [bottom_back_left; bottom_front_left];
plot3(temp(:,1),temp(:,2),temp(:,3),'r','linewidth',3);
temp = [bottom_back_left; bottom_front_right];
plot3(temp(:,1),temp(:,2),temp(:,3),'r','linewidth',3);