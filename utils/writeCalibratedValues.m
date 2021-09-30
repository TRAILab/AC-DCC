function writeCalibratedValues(dcc_obj, opt_problem)

%% Description
% This function writes the calibration values to the file 

format short;

zero_limit = 1e-5;

calibrated_file_id = fopen(dcc_obj.data_files.optimized_params_file_path,'w');
parameter_container = opt_problem.parameter_container;
link_structs = dcc_obj.link_struct;

% Get the theta values
opt_theta_values = zeros(length(dcc_obj.good_meas_idxs), sum(dcc_obj.optimize_theta_flag_vec));
for i=1:length(parameter_container.parameter_list)
    parameter = parameter_container.parameter_list{i};
    if contains(parameter.key, 'theta')
        parameter_key_list = split(parameter.key,'_');
        meas_num = str2double(parameter_key_list{3});
        theta_num = str2double(parameter_key_list{4});
        opt_theta_values(meas_num, theta_num) = parameter_container.getKeyValue(parameter.key);
    end
end
opt_theta_values = rad2deg(opt_theta_values);
if size(opt_theta_values) == size(dcc_obj.encoder_collection)
    theta_errors = opt_theta_values - dcc_obj.encoder_collection;
    theta_offsets_calc = mean(wrapTo180(theta_errors));
    theta_offsets_calc(abs(theta_offsets_calc)<zero_limit)=0;
else
    theta_offsets_calc = zeros(1,dcc_obj.num_DH_links + dcc_obj.use_modified_DH_flag);
end

name_list = containers.Map;
angle_params = containers.Map;
name_list('mdh') = {'theta','d','r','alpha','beta','y'};
angle_params('mdh') = [1,4,5];
name_list('dh') = {'theta','d','r','alpha'};
angle_params('dh') = [1,4];
name_list('4dof') = {'rx','ry','tx','ty'};
angle_params('4dof') = [1,2];
name_list('6dof') = {'T'};
angle_params('6dof') = [1,2,3];

fprintf(calibrated_file_id, "%s\n","start:");

for i=1:length(link_structs)
    link_struct = link_structs(i);
    if i==2
        fprintf(calibrated_file_id, "%s\n", num2str(dcc_obj.num_DH_links));
    end
    index_map = link_struct.index_map;
    type = link_struct.type;
    cam_name = link_struct.cam_name;
    param_names = name_list(type);
    fprintf(calibrated_file_id, "%s",strcat(type,", "));
    temp_num = num2str(i);
    if ~isempty(cam_name) && strcmp(cam_name(1),'S')
        temp_num = cam_name(2:end);
    end
    if strcmp(type,'6dof')
        key_str = strcat('T_', cam_name, 'B');
        param_value = parameter_container.getKeyValue(key_str);
        param_vec_value = tran2vec2(param_value);
        param_vec_value = [rad2deg(param_vec_value(1:3))' param_vec_value(4:6)'];
        param_vec_value(abs(param_vec_value)<zero_limit)=0;
        for j=1:length(param_vec_value)
            param_value = param_vec_value(j);
            fprintf(calibrated_file_id, "%s",strcat(num2str(param_value)," "));
        end
        index_map = zeros(1,6);
    else
        for idx=1:length(index_map)
            param_value = '0';
            if index_map(idx)>0
               key_str = strcat(type, '_', param_names{idx}, '_', temp_num);
               param_value = parameter_container.getKeyValue(key_str);
               if ismember(idx, angle_params(type))
                   param_value = rad2deg(param_value);
               end
            end
            param_value(abs(param_value)<zero_limit)=0;
            fprintf(calibrated_file_id, "%s",strcat(num2str(param_value)," "));
        end
    end
    if ~isempty(link_struct.offset)
        offset_diff = rad2deg(link_struct.offset) + theta_offsets_calc(i);
        offset_diff(abs(offset_diff)<zero_limit)=0;
        fprintf(calibrated_file_id, "%s",strcat(num2str(offset_diff)," "));
    end
    if contains(link_struct.type,'dh')
        fprintf(calibrated_file_id, "%s",strcat(num2str([1 zeros(1,length(index_map)-1)])," "));
    else
        fprintf(calibrated_file_id, "%s",strcat(num2str(zeros(1,length(index_map)))," "));
    end
    fprintf(calibrated_file_id, "%s\n",strcat(num2str(rad2deg(link_struct.bounds))," "));
end

fprintf(calibrated_file_id,"%s","end:");
fclose(calibrated_file_id);
a = 1;