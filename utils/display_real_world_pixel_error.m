function display_real_world_pixel_error(simulation_object, opt_problem, measurement_set, measurement_avg_pixel_error)

figure(9);
clf;
[values, indices] = maxk(measurement_avg_pixel_error, 9);
for k=1:length(values)
    meas_num = indices(k);
    meas_are = values(k);
    subplot(3,3,k);
    measurement_struct = measurement_set{meas_num};
    measurement_struct.num = meas_num;
    measurement_struct.total_num = length(measurement_set);
    imshow(strcat(simulation_object.data_files.folder_path, simulation_object.data_files.real_image_path, simulation_object.camera_intrinsics(1).name, '_', num2str(simulation_object.real_image_mapping(num2str(meas_num))),'.jpeg'));
    title(['Img=',num2str(meas_num),', orig num=',num2str(simulation_object.real_image_mapping(num2str(meas_num))),', avg=',num2str(meas_are)]);
    [~, ~, ~, projected_pix] = DCCResidualOneWayPoseLoop(opt_problem.parameter_container, measurement_struct, simulation_object);
    xs = projected_pix(:,1);
    ys = projected_pix(:,2);
    resx = simulation_object.camera{1}.intrinsics.width;
    resy = simulation_object.camera{1}.intrinsics.height;
    pix_ind_plane = find(xs>0 & xs<resx & ys>0 & ys<resy);
    pix_seen = projected_pix(pix_ind_plane,:);
    hold on;
    scatter(pix_seen(:,1),pix_seen(:,2),'rx');
end
suptitle('Worst Avg Error');

figure(7);
scatter3(simulation_object.encoder_collection(:,1), simulation_object.encoder_collection(:,2), simulation_object.encoder_collection(:,3), 'r', 'filled');
xlabel('Pitch');
ylabel('Roll');
zlabel('Yaw');
title('All Encoder angles (Red), Worst pix RE (black)');
hold on;
scatter3(simulation_object.encoder_collection(indices,1), simulation_object.encoder_collection(indices,2), simulation_object.encoder_collection(indices,3), 'k', 'filled')

figure(10);
clf;
[values, indices] = mink(measurement_avg_pixel_error, 9);
for k=1:length(values)
    meas_num = indices(k);
    meas_are = values(k);
    subplot(3,3,k);
    measurement_struct = measurement_set{meas_num};
    measurement_struct.num = meas_num;
    measurement_struct.total_num = length(measurement_set);
    imshow(strcat(simulation_object.data_files.folder_path,simulation_object.data_files.real_image_path, simulation_object.camera_intrinsics(1).name,'_',num2str(simulation_object.real_image_mapping(num2str(meas_num))),'.jpeg'));
    title(['Img=',num2str(meas_num),', orig num=',num2str(simulation_object.real_image_mapping(num2str(meas_num))),', avg=',num2str(meas_are)]);
    [~, ~, ~, projected_pix] = DCCResidualOneWayPoseLoop(opt_problem.parameter_container, measurement_struct, simulation_object);
    xs = projected_pix(:,1);
    ys = projected_pix(:,2);
    resx = simulation_object.camera{1}.intrinsics.width;
    resy = simulation_object.camera{1}.intrinsics.height;
    pix_ind_plane = find(xs>0 & xs<resx & ys>0 & ys<resy);
    pix_seen = projected_pix(pix_ind_plane,:);
    hold on;
    scatter(pix_seen(:,1),pix_seen(:,2),'rx');
end
suptitle('Best Avg Error');
