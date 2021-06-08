function [ residual_vector_whiten,J_total,residual_vector_reshape] = DCCResidualOneWayReprojection(parameter_container,measurement_set, configuration_object)
%GIMBAL_REPROJECTION_RESIDUAL_BUILDER Summary of this function goes here
%   Detailed explanation goes here

show_plot=0;

[T_S_M, transform_chain] = movingToStaticChain(parameter_container, measurement_set.theta_vec, configuration_object);

% Transform static camera point into gimbal frame
s_camera_pts = applyTransform(T_S_M, measurement_set.camera_points{2});

% Project these points onto the gimbal image plane
all_s_proj_pixels = general_camera_projection(configuration_object.camera_intrinsics(1), s_camera_pts);

% Get the indices of pixels that fall on the image plane
s_indices_on_plane = get_pixel_indices_on_image(all_s_proj_pixels, s_camera_pts, configuration_object.camera_intrinsics(1));

% Select only those pixels that fall on the plane
s_proj_pix_on_plane = all_s_proj_pixels(s_indices_on_plane,:);

% Get the actual gimbal pixel measurements from the measurement set
all_s_cam_pix = measurement_set.pixel_measurements{1};

% Get the corresponding pixels at the indices
s_cam_pix_on_plane = all_s_cam_pix(s_indices_on_plane,:);

% Calculate residual vector
residual_vector = (s_proj_pix_on_plane - s_cam_pix_on_plane);
residual_vector_reshape = reshape(residual_vector',size(residual_vector,1)*2,1);

% Calculate the Jacobian
J_total = [];
residual_vector_whiten = [];
m_camera_points_on_plane = measurement_set.camera_points{2}(s_indices_on_plane,:);
num_points = size(m_camera_points_on_plane,1);
for i=1:num_points
    current_point = m_camera_points_on_plane(i,:)';
    [J_chain,J_scale_offset] = movingToStaticChainJacobianReprojection(parameter_container,measurement_set.theta_vec,configuration_object,transform_chain,current_point);
    % Projection Jacobian (We are projecting onto static camera)
    static_camera = configuration_object.camera{1};
    point_in_static =  applyTransform(T_S_M,current_point');
    [~,J_proj] = static_camera.projectAndJacobian(point_in_static);
    
    J_row_block = [J_chain J_scale_offset];
    
    % perform Jacobian whitening
    current_pixel = residual_vector(i,:);
    pixel_covariance = 0.5*eye(2);
    pixel_information = inv(pixel_covariance);
   
    
    [ r_whiten,J_whiten ] = prewhiten_residual(current_pixel',J_proj*J_row_block,pixel_information );
    residual_vector_whiten = [residual_vector_whiten;r_whiten];
    J_total = [J_total;J_whiten];
end

%optionally plot the reprojection error
if(show_plot && num_points>0)
    plotReprojection(s_cam_pix_on_plane,s_proj_pix_on_plane,configuration_object.camera_intrinsics(1))
end

end

