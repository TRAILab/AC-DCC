function showObjAndPix(sim_obj, encoder_angles)

%% Description:
% This function displays the object and pixel location on the image plane

% Store Variables
target_pts_world = sim_obj.target_pts_world;
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
        target_pts_in_cam_frame = applyTransform(inv(T_WC_list{j}), target_pts_world);
        all_cam_pixels = sim_obj.cameras{j}.project(target_pts_in_cam_frame);
        all_noisy_cam_pixels = addNoise(all_cam_pixels, 'pix', sim_obj.pixel_noise, []);
        cam_indices_on_plane = sim_obj.cameras{j}.getIndicesOnImage(all_noisy_cam_pixels, target_pts_in_cam_frame);
        noisy_cam_pix_on_plane = all_noisy_cam_pixels(cam_indices_on_plane,:);
        ax_list(j) = sim_obj.cameras{j}.plotPixels(noisy_cam_pix_on_plane);
        sim_obj.cameras{j}.plotFOV(T_WC_list{j});
        
        % This is mainly for debug purposes, to see which 3D points the
        % gimbal actually sees
        if j==1 % we want to see what the target points that the gimbal sees
            debug_pts_seen_in_gimbal = cam_indices_on_plane;
        end
    end
    
    title_str = 'Dynamic Camera Cluster';
    callFigure(title_str);
    hold on;
    scatter3(target_pts_world(debug_pts_seen_in_gimbal,1), target_pts_world(debug_pts_seen_in_gimbal,2), target_pts_world(debug_pts_seen_in_gimbal,3), 'filled','r');
    
    % Display all pixels in single plot
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