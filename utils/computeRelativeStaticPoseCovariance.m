function [T_S1_S2_cov] = computeRelativeStaticPoseCovariance(configuration_object, measurement_struct)

% If static1 then tr_flag = 1, but it is actually second cam therefore +1
% same for static2, tr_flag = 2, but it is actually third cam

% only going to optimize over three parameters
parameter_container = ParameterContainer;
parameter_container.parameter_key_map = containers.Map;

T_S2_WS2 = Transformation([0 0 0 0 0 0]);
T_S2_WS2.matrix = measurement_struct.camera_T_target{3};
O1 = OptimizationParameter(T_S2_WS2, 6, 'T_S2_WS2');

T_S1_WS1 = Transformation([0 0 0 0 0 0]);
T_S1_WS1.matrix = measurement_struct.camera_T_target{2};
O2 = OptimizationParameter(T_S1_WS1, 6, 'T_S1_WS1');

T_S1_S2 = Transformation([0 0 0 0 0 0]);
T_S1_S2.matrix = measurement_struct.camera_T_target{2}/measurement_struct.camera_T_target{3}; %measurement_struct.T_S_M{cam_num-1};
O3 = OptimizationParameter(T_S1_S2, 6, 'T_S1_S2');

parameter_container.addParameter(O1);
parameter_container.addParameter(O2);
parameter_container.addParameter(O3);

% set up the optimization problem
opt_problem = Problem(parameter_container);

% add the reprojection residuals from the moving camera
num_static2_points = length(measurement_struct.raw_camera_points{3});
for i=1:num_static2_points
    measured_point = measurement_struct.raw_camera_points{3}(i,:);
    measured_pixel = measurement_struct.raw_pixel_measurements{3}(i,:);
    ms = {measured_point measured_pixel};
    static2_camera = configuration_object.camera{3};
    opt_problem.addResidual('UnaryReprojection', [1], ms, static2_camera);
end

% add the reprojection residuals from the static camera
num_static1_points = length(measurement_struct.raw_camera_points{2});
for i=1:num_static1_points
    measured_point = measurement_struct.raw_camera_points{2}(i,:);
    measured_pixel = measurement_struct.raw_pixel_measurements{2}(i,:);
    ms = {measured_point measured_pixel};
    static1_camera = configuration_object.camera{2};
    opt_problem.addResidual('UnaryReprojection',[2], ms, static1_camera);
end

opt_problem.addDCCCovariancePoseloopResidual();

% Get the relative pose covariance
system_covariance = opt_problem.getSystemCovariance();
T_S1_S2_cov = system_covariance(13:end,13:end); % <-- covariance wrt T_S_M

end

