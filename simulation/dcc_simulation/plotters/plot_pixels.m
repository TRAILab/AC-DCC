function [ax_list, f] = plot_pixels(gimbal_pixels, static_pixels_struct, camera_objects)

% Given pixels in each camera along with the camera objects, return the
% indices of the pixels that fall on the image plane and plot those pixels
f = figure(2);
%set(f, 'Visible', 'off');
clf()

ax_list = {};

for i=1:length(camera_objects)
    ax_list(i) = {subplot(1,length(camera_objects), i)};
end

%ax1 = subplot(1,length(camera_objects),1);
%ax2 = subplot(1,length(camera_objects),2);
%if length(camera_objects)>2
%    ax3 = subplot(1,length(camera_objects),3);
%else
%    ax3 = [];
%end

% Gimbal Camera
if(~isempty(gimbal_pixels))
    resx1 = camera_objects(1).width;
    resy1 = camera_objects(1).height;
    subplot(1,length(camera_objects),1); 
    hold on;
    scatter(gimbal_pixels(:,1),-1*gimbal_pixels(:,2),'filled','r');
    scatter(gimbal_pixels(1,1),-1*gimbal_pixels(1,2),'filled','g');
    xlim([0 resx1])
    ylim([-resy1 0])
    title(camera_objects(1).name)
    pbaspect([4 3 1])
end

% Static Camera 1
if(~isempty(static_pixels_struct{1}))
    static_pixels = static_pixels_struct{1};
    resx2 = camera_objects(2).width;
    resy2 = camera_objects(2).height;
    subplot(1,length(camera_objects),2);
    hold on;
    scatter(static_pixels(:,1),-1*static_pixels(:,2),'filled','b');
    scatter(static_pixels(1,1),-1*static_pixels(1,2),'filled','g');
    xlim([0 resx2])
    ylim([-resy2 0])
    title(camera_objects(2).name)
    pbaspect([4 3 1])
end

% More than one static camera 
if length(camera_objects)>2 
    if(~isempty(static_pixels_struct{2}))
        static_pixels = static_pixels_struct{2};
        resx2 = camera_objects(3).width;
        resy2 = camera_objects(3).height;
        subplot(1,length(camera_objects),3);
        hold on;
        scatter(static_pixels(:,1),-1*static_pixels(:,2),'filled','b');
        scatter(static_pixels(1,1),-1*static_pixels(1,2),'filled','g');
        xlim([0 resx2])
        ylim([-resy2 0])
        title(camera_objects(3).name)
        pbaspect([4 3 1])
    end
end