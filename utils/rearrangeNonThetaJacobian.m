function J_rearranged = rearrangeNonThetaJacobian(dcc_obj, J_input)

list_num_opt_params = [];
num_dh_links = dcc_obj.num_DH_links;
static_cam_idx = str2double(dcc_obj.static_cam_key(4));
num_thetas_to_opt = sum(dcc_obj.optimize_theta_flag_vec);

for i=1:length(dcc_obj.link_struct)
    idx_map = dcc_obj.link_struct(i).index_map;
    list_num_opt_params = [list_num_opt_params, length(idx_map(idx_map~=-1))];
end

J_rearranged = zeros(size(J_input,1), sum(list_num_opt_params));
num_of_non_static_params = sum(list_num_opt_params(1:1+num_dh_links)); % This stores the number of parameters that do not belong to the static cameras
J_rearranged(:,1:num_of_non_static_params) = J_input(:,1:num_of_non_static_params);

J_thetas = J_input(:,end-num_thetas_to_opt+1:end);

start_input_col = num_of_non_static_params + 1;
end_input_col = start_input_col + list_num_opt_params(1+num_dh_links+static_cam_idx)-1;

idx_being_analyzed = 1 + num_dh_links + static_cam_idx;
start_output_col = sum(list_num_opt_params(1:idx_being_analyzed-1))+1;
end_output_col = sum(list_num_opt_params(1:idx_being_analyzed));

J_rearranged(:,start_output_col:end_output_col) = J_input(:, start_input_col:end_input_col);
J_rearranged = [J_rearranged J_thetas];