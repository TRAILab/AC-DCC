function noisy_vals = addNoise(orig_value, noise_type, noise_vals, theta_flag_vec)
%function noisy_vals = add_noise(orig_value, noise_type, theta_flag_vec, noise_bounds)

% Add gaussian noise to values of a certain mean and variance, based on the
% standard deviation.

if strcmp(noise_type, 'transformation')
    
    trans_noise = noise_vals.trans;
    rot_noise = noise_vals.rot;
    trans_noise_vec = normrnd(trans_noise.mean, trans_noise.std_dev, 1, 3);
    rot_noise_vec = normrnd(rot_noise.mean, rot_noise.std_dev, 1, 3);
    noisy_transformation = Transformation([rot_noise_vec, trans_noise_vec]).matrix;
    noisy_vals = noisy_transformation*orig_value;
    
else
    
    num_vals = size(orig_value,1);
    num_dim = size(orig_value,2);

    % Generate random numbers 
    rng('shuffle');
    noise = normrnd(noise_vals.mean, noise_vals.std_dev, num_vals, num_dim);

    % Get noise for values. 
    % Rand value in range = (b-a)*point-a.
    % noise = (noise_bounds(2)-noise_bounds(1)).*rand(num_vals,num_dim)+noise_bounds(1);

    if strcmp(noise_type, 'enc')
        noisy_vals = orig_value + noise.*theta_flag_vec;
        %noisy_vals = wrapToPi(noisy_vals);
    elseif strcmp(noise_type, 'pix')
        noisy_vals = orig_value + noise;
    end
    
end