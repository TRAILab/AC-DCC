clear all
clc
close all

% This file is used to generate different environments

% Cube
hs = 2; % half side size
sep = 0.2;
pts_range = -hs:sep:hs;
pts_range = pts_range';
const_val = hs*ones(length(pts_range),1);

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

scatter3(target_pts(:,1),target_pts(:,2), target_pts(:,3),'filled','k');