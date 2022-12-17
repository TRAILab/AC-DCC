function calcParamDiff(filepath1, filepath2)

%% This file calculates the difference in rotation and translation
% parameters if we know the true values

angle_params = containers.Map;
trans_params = containers.Map;
angle_params('mdh') = [4,5];
trans_params('mdh') = [2,3,6];
angle_params('dh') = 4;
trans_params('dh') = [2,3];
angle_params('4dof') = [1,2];
trans_params('4dof') = [3,4];
angle_params('6dof') = [1,2,3];
trans_params('6dof') = [4,5,6];

true_params_path = strcat(filepath1);
true_link_structs = loadLinkParams(true_params_path);

calibrated_params_path = strcat(filepath2);
calibrated_link_structs = loadLinkParams(calibrated_params_path);

angle_diffs = [];
trans_diffs = [];

for i=1:length(true_link_structs)
    true_dvs = true_link_structs(i).default_values;
    calibrated_dvs = calibrated_link_structs(i).default_values;
    if strcmp(true_link_structs(i).type,'6dof')
        true_tr = vec2tran2(true_dvs');
        calibrated_tr = vec2tran2(calibrated_dvs');
        diff_tr = true_tr\calibrated_tr;
        diff_dvs = tran2vec2(diff_tr)';
    else
        diff_dvs = abs(true_dvs-calibrated_dvs);
    end
    angle_diffs = [angle_diffs; diff_dvs(angle_params(true_link_structs(i).type))'];
    trans_diffs = [trans_diffs; diff_dvs(trans_params(true_link_structs(i).type))'];
end
mr = mean(abs(rad2deg(angle_diffs)));
sr = std(abs(rad2deg(angle_diffs)));
mt = mean(abs(trans_diffs));
st = std(abs(trans_diffs));
disp(strcat('Parameter Rotation Error (Deg): Mean=', num2str(mr),' and Std Dev=', num2str(sr)));
disp(strcat("Parameter Translation Error (m): Mean=", num2str(mt)," and Std Dev=",num2str(st)));