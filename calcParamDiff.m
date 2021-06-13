function calcParamDiff(data_files)

name_list = containers.Map;
angle_params = containers.Map;
trans_params = containers.Map;
name_list('mdh') = {'theta','d','r','alpha','beta','y'};
angle_params('mdh') = [4,5];
trans_params('mdh') = [2,3,6];
name_list('dh') = {'theta','d','r','alpha'};
angle_params('dh') = 4;
trans_params('dh') = [2,3];
name_list('4dof') = {'rx','ry','tx','ty'};
angle_params('4dof') = [1,2];
trans_params('4dof') = [3,4];
name_list('6dof') = {'T'};
angle_params('6dof') = [1,2,3];
trans_params('6dof') = [4,5,6];

true_params_path = strcat(data_files.true_params_file_path);
true_link_structs = loadLinkParams(true_params_path);

calibrated_params_path = strcat(data_files.folder_path,'calibratedParams.txt');
calibrated_link_structs = loadLinkParams(calibrated_params_path);

angle_diffs = [];
trans_diffs = [];

for i=1:length(true_link_structs)
    true_dvs = true_link_structs(i).default_values;
    calibrated_dvs = calibrated_link_structs(i).default_values;
    diff_dvs = abs(true_dvs-calibrated_dvs);
    angle_diffs = [angle_diffs; diff_dvs(angle_params(true_link_structs(i).type))'];
    trans_diffs = [trans_diffs; diff_dvs(trans_params(true_link_structs(i).type))'];
end
mr = mean(abs(rad2deg(angle_diffs)));
sr = std(abs(rad2deg(angle_diffs)));
mt = mean(abs(trans_diffs));
st = std(abs(trans_diffs));
disp(strcat('Rotation Error (Deg): Mean=', num2str(mr),' and Std Dev=', num2str(sr)));
disp(strcat("Translation Error (m): Mean=", num2str(mt)," and Std Dev=",num2str(st)));