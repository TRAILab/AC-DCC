function [] = plotReprojection(measured_pixels,reprojected_pixels,camera_object)

% plots the reprojection error, helps with debugging
% assumes that measured_pixels and reprojected_pixels are 2 columns(one for each pixel
% dimension), by N rows.

resx = camera_object.width*10;
resy = camera_object.height*10;

figure;
A = measured_pixels;
scatter(A(:,1),A(:,2),[],'b','o');
hold on
B = reprojected_pixels;
scatter(B(:,1),B(:,2),[],'r','x');
xlim( [0 resx]);
ylim( [0 resy]);
axis equal;
grid on;
set(gca,'YDir','Reverse')
%plot correspondence lines and compute total error

for(i=1:length(A))
    p1 = [A(i,1) B(i,1)];
    p2 = [A(i,2) B(i,2)];
    plot(p1,p2 ,'g-', 'linewidth',2);
    axis equal;
end

end

