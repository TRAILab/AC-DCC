function pts = getValuesInRange(low, high, num_pts)

%% Description
% This returns a vector of values in range of size num_pts = [rows cols]

num_rows = num_pts(1);
num_cols = num_pts(2);
pts = zeros(num_rows, num_cols);

for c=1:num_cols
    pts(:,c) = (high-low).*rand(num_rows,1) + low;
end