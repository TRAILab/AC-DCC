function [] = plotGimbalAndStaticCamera(measurement_struct)
% plot the poseloop
figure;
hold on;
grid on;
T_WC = eye(4);
color_num = 0;

T_SG = measurement_struct.T_S_M;
T_SC  = measurement_struct.camera_T_target{1};
T_GC  = measurement_struct.camera_T_target{2};

% Plot Gimbal camera
color_num = color_num+1;
T_WG = T_WC*inv(T_SC)*T_SG;
plotAxis(T_WG,0.2, 'b','T^d')

% Plot static camera
color_num = color_num+1;
T_WS = T_WC*inv(T_GC)*inv(T_SG);
plotAxis(T_WS,0.2, 'r','T^s')

% plot chessboard frame (static)
color_num = color_num+1;
plotAxis(T_WC,0.2, 'k','T^{cs}')

axis equal;
end

