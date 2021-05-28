function output_pts = applyTransform(T, input_pts)

% Returns non-homogeneous points after transforming with T 

if size(input_pts,2)==3
    input_pts = [input_pts ones(size(input_pts,1),1)];
end

output_pts = T*input_pts';

output_pts = output_pts(1:3,:)';