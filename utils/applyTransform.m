function output_pts_B = applyTransform(T_BA, input_pts_A)

%% Description
% Transform point from A to B

if size(input_pts_A,2)==3
    input_pts_A = [input_pts_A ones(size(input_pts_A,1),1)];
end

output_pts_B = T_BA*input_pts_A';

output_pts_B = output_pts_B(1:3,:)';