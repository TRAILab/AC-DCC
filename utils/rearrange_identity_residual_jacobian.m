function J_rearranged = rearrange_identity_residual_jacobian(simulation_object, J_input)

list_params_opt = [];
num_dh_links = length(simulation_object.link_struct) - length(simulation_object.camera);

for i=1:length(simulation_object.link_struct)
    idx_map = simulation_object.link_struct(i).index_map;
    list_params_opt = [list_params_opt, length(idx_map(idx_map~=-1))];
end

J_rearranged = zeros(6, sum(list_params_opt)+sum(simulation_object.optimize_theta_flag_vec));
num_of_non_static_params = sum(list_params_opt(1:1+num_dh_links));

% For Static camera 1
start_input_col = 1;
end_input_col = start_input_col + list_params_opt(1+num_dh_links+1) - 1;
start_output_col = num_of_non_static_params + 1;
end_output_col = start_output_col + list_params_opt(1+num_dh_links+1) - 1;
J_rearranged(:, start_output_col:end_output_col) = J_input(:, start_input_col:end_input_col);

% For Static camera 2
start_input_col = end_input_col + 1;
end_input_col = start_input_col + list_params_opt(1+num_dh_links+2) - 1;
start_output_col = end_output_col + 1;
end_output_col = start_output_col + list_params_opt(1+num_dh_links+2) - 1;
J_rearranged(:,start_output_col:end_output_col) = J_input(:, start_input_col:end_input_col);