function plotRealWorldImages(dcc_obj, opt_problem, measurement_set, avg_pixel_error)

real_image_path = dcc_obj.data_files.real_image_path;
optimize_theta_flag_vec = dcc_obj.optimize_theta_flag_vec;

% Plot the error on the static camera images
for c=1:length(dcc_obj.cameras)-1
    sensor_name = dcc_obj.cameras{c+1}.sensor_name;
    dcc_obj.static_cam_key = strcat('T_S',num2str(c),'B');
    callFigure(strcat("Real World: ", sensor_name));
    clf;
    avg_pe = avg_pixel_error(:,c);
    [max_avg_pe, top_worse_idxs] = maxk(avg_pe,9);
    real_image_idxs = getModifiedIdxs(dcc_obj.bad_meas_idxs, top_worse_idxs);
    for m=1:9
        top_worse_idx = top_worse_idxs(m);
        subplot(3,3,m);
        imshow(strcat(real_image_path, num2str(real_image_idxs(m)),'_', sensor_name, '.jpeg'));
        title(strcat("Image ", num2str(real_image_idxs(m)), " RE: ", num2str(max_avg_pe(m))));

        for i=1:length(optimize_theta_flag_vec)
            if optimize_theta_flag_vec(i)==1
                if i==1 && dcc_obj.use_modified_DH_flag
                    key = strcat('mdh_theta_', int2str(top_worse_idx), '_', int2str(i));
                else
                    key = strcat('dh_theta_', int2str(top_worse_idx), '_', int2str(i));
                end
                joint_angles(i) = opt_problem.parameter_container.getKeyValue(key);
            end
        end
        
        [T_SM_est_mat, ~] = movingToStaticChain(opt_problem.parameter_container, joint_angles, dcc_obj);
        T_SW_est = T_SM_est_mat*measurement_set{top_worse_idx}.T_CW{1};
        static_pts = applyTransform(T_SW_est, measurement_set{top_worse_idx}.target_points{c+1});
        pixels = dcc_obj.cameras{c+1}.project(static_pts);
        hold on;
        scatter(pixels(:,1),pixels(:,2),'rx');
    end
end

% Plot the gimbal images
callFigure(strcat("Real World: ", dcc_obj.cameras{1}.sensor_name));
clf;
sensor_name = dcc_obj.cameras{1}.sensor_name;
for m=1:9
    subplot(3,3,m);
    imshow(strcat(real_image_path, num2str(real_image_idxs(m)),'_', sensor_name, '.jpeg'));
    title(strcat("Image ", num2str(real_image_idxs(m))));
end