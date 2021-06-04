function showObjAndPix(simulation_object, encoder_angles)

% Store Variables
world_T_target = simulation_object.transforms.world_T_target;
target_pts = simulation_object.target_pts;
encoder_angles_rad = encoder_angles;

opt_problem = setupOptimizationProblem(simulation_object, []);

for i=1:size(encoder_angles_rad,1)
    
    % Display the object. Returns transforms for use at other places
    T_WC_list = displaySimulationObject(simulation_object, encoder_angles_rad(i,:), opt_problem);
    
    % Create figures for points that fall on the camera frames
    for j=1:length(simulation_object.cameras)
        % Get pixels that fall on the image planes
        target_pts_in_cam_frame = applyTransform(T_WC_list{j}\world_T_target, target_pts);
        all_cam_pixels = simulation_object.cameras{j}.project(target_pts_in_cam_frame);
        all_noisy_cam_pixels = addNoise(all_cam_pixels, 'pix', simulation_object.pixel_noise, []);
        cam_indices_on_plane = simulation_object.cameras{j}.getIndicesOnImage(all_noisy_cam_pixels, target_pts_in_cam_frame);
        noisy_cam_pix_on_plane = all_noisy_cam_pixels(cam_indices_on_plane,:);
        ax_list(j) = simulation_object.cameras{j}.plotPixels(noisy_cam_pix_on_plane);
    end
    
    % Display all figures in single plot
    if isempty(findobj('type','figure','name','all_pixels'))
        figure('Name','all_pixels');
    else
        figure(findobj('type','figure','name','all_pixels').Number);
    end
    clf; 
    num_rows = ceil(length(simulation_object.cameras)/3);
    for j=1:length(simulation_object.cameras)
        s = subplot(num_rows, 3, j);
        c = get(ax_list(j), 'children');
        copyobj(c, s);
        title(ax_list(j).Title.String);
        xlim(ax_list(j).XLim);
        ylim(ax_list(j).YLim);
        pbaspect(ax_list(j).PlotBoxAspectRatio);
    end
  
end