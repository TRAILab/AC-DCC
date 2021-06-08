function combine_figures(ax1, ax2, ax3, ax4, camera_objects, title_str)
%COMBINE_FIGURES Combine the two previously created figures into one figure
%   Combines the created figures for the transformation and the pixel
%   measurements for a side by side comparison

global comb_fig
comb_fig = figure(3);
figure(comb_fig)
[az el] = view();
clf(comb_fig)

% Create a new figure
%h3 = figure('Name','combined_figs'); 

% Get subplot handles
s1 = subplot(2, 2, 1); 
%title({title_str{1}; title_str{2}; title_str{3}})

% Plot the gimbal pixels
s2 = subplot(2, 2, 2);
resx1 = camera_objects(1).width;
resy1 = camera_objects(1).height;
xlim([0 resx1])
ylim([-resy1 0])
title(camera_objects(1).name)
pbaspect([4 3 1])

% Plot static 1 pixels
s3 = subplot(2, 2, 3);
resx2 = camera_objects(2).width;
resy2 = camera_objects(2).height;
xlim([0 resx2])
ylim([-resy2 0])
title(camera_objects(2).name)
pbaspect([4 3 1])


% Get handles to the children in the figure
fig1 = get(ax1, 'children');
fig2 = get(ax2, 'children');
fig3 = get(ax3, 'children');

% Copy the figure children to the subplots
copyobj(fig1, s1);
copyobj(fig2, s2);
copyobj(fig3, s3);

% Plot static 2 pixels
if length(camera_objects)>2
    s4 = subplot(2, 2, 4);
    resx2 = camera_objects(3).width;
    resy2 = camera_objects(3).height;
    xlim([0 resx2])
    ylim([-resy2 0])
    title(camera_objects(3).name)
    pbaspect([4 3 1])
    fig4 = get(ax4, 'children');
    copyobj(fig4, s4);
end

end

