function plotErrorStats(dcc_obj, avg_error)

avg_pixel_error = avg_error.avg_pixel_error;
avg_rot_error = avg_error.avg_rot_error;
avg_trans_error = avg_error.avg_trans_error;

% Plot the Pixel errors
callFigure('Avg Pixel Error');
clf;
for i=1:size(avg_pixel_error,2)
    temp_measurement_avg_pixel_error = avg_pixel_error(:,i);
    all_meas_set_avg_pixel_error = mean(temp_measurement_avg_pixel_error);
    subplot(size(avg_pixel_error,2),1,i);
    bar(temp_measurement_avg_pixel_error);
    title(['Avg pix error for each measurement set ', num2str(all_meas_set_avg_pixel_error),'(',dcc_obj.cameras{i+1}.sensor_name,')']);
    hold on;
    plot(xlim,[all_meas_set_avg_pixel_error all_meas_set_avg_pixel_error], 'r')
    xticks(1:2:length(avg_pixel_error));
end

if ~dcc_obj.reproj_error_formulation
    % Plot the Rotational errors
    callFigure('Avg Rotation Error');
    clf;
    for i=1:size(avg_rot_error,2)
        temp_measurement_avg_rot_error = avg_rot_error(:,i);
        all_meas_set_avg_rot_error = mean(temp_measurement_avg_rot_error);
        subplot(size(avg_rot_error,2),1,i);
        bar(temp_measurement_avg_rot_error);
        title(['Avg rot error for each measurement set ', num2str(all_meas_set_avg_rot_error),'(',dcc_obj.cameras{i+1}.sensor_name,')']);
        hold on;
        plot(xlim,[all_meas_set_avg_rot_error all_meas_set_avg_rot_error], 'r')
        xticks(1:2:length(avg_rot_error));
    end

    % Plot the Trans errors
    callFigure('Avg Translation Error');
    clf;
    for i=1:size(avg_trans_error,2)
        temp_measurement_avg_trans_error = avg_trans_error(:,i);
        all_meas_set_avg_trans_error = mean(temp_measurement_avg_trans_error);
        subplot(size(avg_trans_error,2),1,i);
        bar(temp_measurement_avg_trans_error);
        title(['Avg trans error for each measurement set ', num2str(all_meas_set_avg_trans_error),'(',dcc_obj.cameras{i+1}.sensor_name,')']);
        hold on;
        plot(xlim,[all_meas_set_avg_trans_error all_meas_set_avg_trans_error], 'r')
        xticks(1:2:length(avg_trans_error));
    end
end