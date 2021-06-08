function J_rearranged = rearrangeThetaJacobian(dcc_obj, J_input, measurement_set)

num_thetas_to_optimize = sum(dcc_obj.optimize_theta_flag_vec);
num_measurement_sets = length(measurement_set);
J_diag_thetas = zeros(size(J_input,1), num_thetas_to_optimize*num_measurement_sets);
num_non_theta = size(J_input,2)-num_thetas_to_optimize;
J_params = J_input(:,1:num_non_theta);
J_thetas = J_input(:,num_non_theta+1:end);
for m=1:num_measurement_sets
    measurement_struct = measurement_set{m};
    empty_counter = 0;
    for c=1:length(measurement_struct.T_SM)
        if isempty(measurement_struct.T_SM{c})
            empty_counter = empty_counter + 1;
        end
    end
    magic_num = length(dcc_obj.cameras)-1-empty_counter+dcc_obj.add_identity_residual;
    input_start_row = 6*magic_num*(m-1)+1;
    input_end_row = 6*magic_num*(m-1)+6*magic_num;
    temp = J_thetas(input_start_row:input_end_row,:);

    output_start_row = 6*magic_num*(m-1)+1;
    output_end_row = 6*magic_num*(m-1)+6*magic_num;
    output_start_col = num_thetas_to_optimize*(m-1)+1;
    output_end_col = num_thetas_to_optimize*(m-1)+num_thetas_to_optimize;
    J_diag_thetas(output_start_row:output_end_row, output_start_col:output_end_col) = temp;
end
J_rearranged = [J_params J_diag_thetas];