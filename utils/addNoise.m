function noisy_vals = addNoise(orig_value, noise_type, noise_vals, theta_flag_vec)

%% Description
% Add gaussian noise to variables based on noise type, if we are dealing
% with joint angles, we need to know which angles to add noise to specified
% in the vector theta_flag_vec

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

    if strcmp(noise_type, 'enc')
        noisy_vals = orig_value + noise.*theta_flag_vec;
        %noisy_vals = wrapToPi(noisy_vals);
    elseif strcmp(noise_type, 'pix')
        noisy_vals = orig_value + noise;
    end
    
end