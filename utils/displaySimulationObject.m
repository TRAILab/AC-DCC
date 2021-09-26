function T_WC_list = displaySimulationObject(sim_obj, encoder_angles, opt_problem)

%% Description:
% This function displays the simulation object and returns a vector cell of
% the different camera to world transforms

title_str = 'Dynamic Camera Cluster';
num_dh_links = sim_obj.num_DH_links;

if isfield(sim_obj,'transforms')
    transforms = sim_obj.transforms;
else
    transforms.world = eye(4);
    transforms.world_T_base = [1 0 0 0; 
                               0 -1 0 0;
                               0 0 -1 0;
                               0 0 0 1];
end

callFigure(title_str);
clf()

colors = ['c','m','r','g','y','b','k','c','y','r','g','b','m','k'];
joint_names = {['L1'],['L2'],['L3'],['L4'],['EE']};

% Plot World transform
color_num = 1;
plotAxis(transforms.world, 0.2, colors(color_num), 'T^w');

% Plot base to world frame transformation
T_WB = transforms.world_T_base;
color_num = color_num+1;
plotAxis(T_WB, 0.2, colors(color_num),'b')

% Plot target
%target_pts_in_world = applyTransform(sim_obj.transforms.world_T_target, sim_obj.target_pts);
target_pts_in_world = sim_obj.target_pts_world;
hold on;
scatter3(target_pts_in_world(:, 1), target_pts_in_world(:, 2), target_pts_in_world(:, 3), 'filled', 'k');

% Setup the optimization problem
if isempty(opt_problem)
    opt_problem = setupOptimizationProblem(sim_obj, []);
end

% Plot the static cameras
for i=1:length(sim_obj.cameras)-1
    sim_obj.static_cam_key = strcat('T_S',num2str(i),'B');
    [~, transform_chain] = movingToStaticChain(opt_problem.parameter_container, encoder_angles, sim_obj);
    color_num = color_num + 1;
    T_SB = transform_chain{end};
    T_WS = T_WB/T_SB;
    T_WS_list{i} = T_WS;
    plotAxis(T_WS, 0.2, colors(color_num), strcat('s',num2str(i)))
end

% Set the first as the base to world.
T_WDHlink = T_WB;
for t=1:num_dh_links
    T_WDHlink = T_WDHlink*(transform_chain{end-t});
    color_num = color_num + 1;
    plotAxis((T_WDHlink), 0.2, colors(color_num),joint_names{t});
end

% Plot Gimbal camera
color_num = color_num+1;
T_WG = T_WDHlink*(transform_chain{1});
plotAxis(T_WG, 0.3, colors(color_num),'d')

% Get all the transforms from cameras to world
T_WC_list = [{T_WG} T_WS_list];

if isfield(sim_obj,'target_pts_world')
    hold on;
    temp_target_pts_world = sim_obj.target_pts_world;
    if ~sim_obj.use_random_points
        scatter3(temp_target_pts_world(:,1), temp_target_pts_world(:,2), temp_target_pts_world(:,3), 'filled','k');
    end
    plotAxis(sim_obj.transforms.world_T_target, 0.2, 'k', 'T');
end
title(title_str);