function show_obj_and_pix(simulation_object, encoder_angles)

% Store Variables
world_T_target = simulation_object.transforms.world_T_target;
target_pts = simulation_object.target_pts;
pixel_std_dev = simulation_object.pixel_noise.std_dev;
encoder_angles_rad = encoder_angles;

for i=1:size(encoder_angles_rad,1)
    
    % Display the object. Returns transforms for use at other places
    [w_T_s_list, w_T_g, ax1, title_str, ~] = display_simulation_object(simulation_object, encoder_angles_rad(i,:),[]);
    
    % Get target pts in static frame. INV(A)*b = A\b
    % Project camera points on image plane
    % Add noise to pixels
    % Get pixels that fall on the plane and corresponding indices of 3D points
    % Get noisy pixels on the image plane
    target_pts_in_gimbal_frame = applyTransform(w_T_g\world_T_target, target_pts);
    all_gimbal_pixels = general_camera_projection(simulation_object.camera_intrinsics(1), target_pts_in_gimbal_frame);
    all_noisy_gimbal_pixels = add_noise(all_gimbal_pixels, 'pix', simulation_object.pixel_noise, []);
    gimbal_indices_on_plane = get_pixel_indices_on_image(all_noisy_gimbal_pixels, target_pts_in_gimbal_frame, simulation_object.camera_intrinsics(1));
    noisy_gimbal_pix_on_plane = all_noisy_gimbal_pixels(gimbal_indices_on_plane,:);
    
    for j=2:length(simulation_object.camera_intrinsics) % Minus one for the gimbal camera
        target_pts_in_static_frame = applyTransform(w_T_s_list{j-1}\world_T_target, target_pts);
        all_static_pixels = general_camera_projection(simulation_object.camera_intrinsics(j), target_pts_in_static_frame);
        all_noisy_static_pixels = add_noise(all_static_pixels, 'pix', simulation_object.pixel_noise, []);
        static_indices_on_plane = get_pixel_indices_on_image(all_noisy_static_pixels, target_pts_in_static_frame, simulation_object.camera_intrinsics(j));
        noisy_static_pix_on_plane{j-1} = all_noisy_static_pixels(static_indices_on_plane,:);
    end
    
    % Plot pixels
    [ax_list] = plot_pixels(noisy_gimbal_pix_on_plane, noisy_static_pix_on_plane, simulation_object.camera_intrinsics);
    
    % Display all figures in single plot
    combine_figures(ax1, ax2, ax3, ax4, simulation_object.camera_intrinsics, title_str);
    %close(f)
end