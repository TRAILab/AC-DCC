function pts = get_3d_pts_in_range(low,high,num_pts)

% Generate 3D points in a certain range

pts = (high-low).*rand(num_pts,1) + low;