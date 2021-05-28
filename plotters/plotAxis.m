function plotAxis(T, scale, color, name, figure_num)

p0 = [0 0 0];
px = [scale 0 0];
py = [0 scale 0];
pz = [0 0 scale];

% transform according to the transformation
p0 = applyTransform(T,p0);
px = applyTransform(T,px);
py = applyTransform(T,py);
pz = applyTransform(T,pz);

p1 = [p0;px];
p2 = [p0;py];
p3 = [p0;pz];

hold on;
plot3(p1(:,1), p1(:,2), p1(:,3), color,'linewidth',2,'DisplayName',name);
plot3(p2(:,1), p2(:,2), p2(:,3), color,'linewidth',2,'linestyle','--');
plot3(p3(:,1), p3(:,2), p3(:,3), color,'linewidth',2,'linestyle',':');
axis equal;

rv = rand(1)*0.01;

text(p0(1)+rv,p0(2)+rv,p0(3)+rv,name);
text(p1(2,1)+rv,p1(2,2)+rv,p1(2,3)+rv,strcat('x_',name(end)));
text(p2(2,1)+rv,p2(2,2)+rv,p2(2,3)+rv,strcat('y_',name(end)));
text(p3(2,1)+rv,p3(2,2)+rv,p3(2,3)+rv,strcat('z_',name(end)));
% text(p1(2,1)+rv,p1(2,2)+rv,p1(2,3)+rv,'x');
% text(p2(2,1)+rv,p2(2,2)+rv,p2(2,3)+rv,'y');
% text(p3(2,1)+rv,p3(2,2)+rv,p3(2,3)+rv,'z');

end

