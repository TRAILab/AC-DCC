function plot_pixel_error_stats(simulation_object, measurement_avg_pixel_error)

figure(8);
clf;

for i=1:size(measurement_avg_pixel_error,1)
    temp_measurement_avg_pixel_error = measurement_avg_pixel_error(i,:);
    all_meas_set_avg_pixel_error = mean(temp_measurement_avg_pixel_error);
    subplot(size(measurement_avg_pixel_error,1),1,i);
    bar(temp_measurement_avg_pixel_error);
    title(['Avg pix error for each measurement set ', num2str(all_meas_set_avg_pixel_error),'(',simulation_object.cameras{i+1}.sensor_name,')']);
    hold on;
    plot(xlim,[all_meas_set_avg_pixel_error all_meas_set_avg_pixel_error], 'r')
    xticks(1:2:length(measurement_avg_pixel_error));
end
%yticks(1:5:max(measurement_avg_pixel_error));