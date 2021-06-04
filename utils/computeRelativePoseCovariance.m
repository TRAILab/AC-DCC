function [cov_mat] = computeRelativePoseCovariance(simulation_object, measurement_struct)

static_cam_idx = str2double(simulation_object.static_cam_key(4));

% only going to optimize over three parameters
parameter_container = ParameterContainer;
parameter_container.parameter_key_map = containers.Map;

T_M_WM = Transformation([0 0 0 0 0 0]);
T_M_WM.matrix = measurement_struct.camera_T_target{1};
O1 = OptimizationParameter(T_M_WM, 6, 'T_M_WM');

T_S_WS = Transformation([0 0 0 0 0 0]);
T_S_WS.matrix = measurement_struct.camera_T_target{static_cam_idx+1};
O2 = OptimizationParameter(T_S_WS, 6, 'T_S_WS');

T_S_M = Transformation([0 0 0 0 0 0]);
T_S_M.matrix = measurement_struct.T_S_M{static_cam_idx};
O3 = OptimizationParameter(T_S_M, 6, 'T_S_M');

parameter_container.addParameter(O1);
parameter_container.addParameter(O2);
parameter_container.addParameter(O3);

% set up the optimization problem
opt_problem = Problem(parameter_container);

% add the reprojection residuals from the moving camera
num_moving_points = length(measurement_struct.raw_camera_points{1});
for i=1:num_moving_points
    measured_point = measurement_struct.raw_camera_points{1}(i,:);
    measured_pixel = measurement_struct.raw_pixel_measurements{1}(i,:);
    ms = {measured_point measured_pixel};
    moving_camera = simulation_object.camera{1};
    opt_problem.addResidual('UnaryReprojection', 1, ms, moving_camera);
end

% add the reprojection residuals from the static camera
num_static_points = length(measurement_struct.raw_camera_points{static_cam_idx+1});
for i=1:num_static_points
    measured_point = measurement_struct.raw_camera_points{static_cam_idx+1}(i,:);
    measured_pixel = measurement_struct.raw_pixel_measurements{static_cam_idx+1}(i,:);
    ms = {measured_point measured_pixel};
    static_camera = simulation_object.camera{static_cam_idx+1};
    opt_problem.addResidual('UnaryReprojection', 2, ms,static_camera);
end

opt_problem.addDCCCovariancePoseloopResidual();

% Get the relative pose covariance
system_covariance = opt_problem.getSystemCovariance();

if simulation_object.pixel_error_formulation
    cov_mat = system_covariance(1:6, 1:6);
else
    cov_mat = system_covariance(13:end, 13:end); % <-- covariance wrt T_S_M
end

end


