function showObjAndPix(sim_obj, encoder_angles)

% Store Variables
world_T_target = sim_obj.transforms.world_T_target;
target_pts = sim_obj.target_pts;
encoder_angles_rad = encoder_angles;

opt_problem = setupOptimizationProblem(sim_obj, []);

for i=1:size(encoder_angles_rad,1)
    
    % Add noise to angles
    disp(strcat("Angle Readings (Deg): ", num2str(rad2deg(encoder_angles_rad))));
    encoder_angles_rad = addNoise(encoder_angles_rad(i,:), 'enc', sim_obj.encoder_noise, sim_obj.optimize_theta_flag_vec);
    disp(strcat("True Angles (Deg): ", num2str(rad2deg(encoder_angles_rad))));
    disp("----------------------------------------------");
    
    % Display the object. Returns transforms for use at other places
    T_WC_list = displaySimulationObject(sim_obj, encoder_angles_rad(i,:), opt_problem);
    
    % Create figures for points that fall on the camera frames
    for j=1:length(sim_obj.cameras)
        % Get pixels that fall on the image planes
        target_pts_in_cam_frame = applyTransform(T_WC_list{j}\world_T_target, target_pts);
        all_cam_pixels = sim_obj.cameras{j}.project(target_pts_in_cam_frame);
        all_noisy_cam_pixels = addNoise(all_cam_pixels, 'pix', sim_obj.pixel_noise, []);
        cam_indices_on_plane = sim_obj.cameras{j}.getIndicesOnImage(all_noisy_cam_pixels, target_pts_in_cam_frame);
        noisy_cam_pix_on_plane = all_noisy_cam_pixels(cam_indices_on_plane,:);
        ax_list(j) = sim_obj.cameras{j}.plotPixels(noisy_cam_pix_on_plane);
    end
    
    % Display all figures in single plot
    if isempty(findobj('type','figure','name','all_pixels'))
        figure('Name','all_pixels');
    else
        figure(findobj('type','figure','name','all_pixels').Number);
    end
    clf; 
    num_rows = ceil(length(sim_obj.cameras)/3);
    for j=1:length(sim_obj.cameras)
        s = subplot(num_rows, 3, j);
        c = get(ax_list(j), 'children');
        copyobj(c, s);
        title(ax_list(j).Title.String);
        xlim(ax_list(j).XLim);
        ylim(ax_list(j).YLim);
        pbaspect(ax_list(j).PlotBoxAspectRatio);
    end
  
end