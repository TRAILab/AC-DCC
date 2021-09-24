function entire_chain_jacobian = getEntireChainJacobian(calibration_vec)

% This function was maily written for observability analysis to make sure
% we have the right analytical form of the jacobians. See handwritten notes

T_IB_params = calibration_vec(1:4);
rx_IB_param = T_IB_params(1);
ry_IB_param = T_IB_params(2);
tx_IB_param = T_IB_params(3);
ty_IB_param = T_IB_params(4);
T_IB_matrix = generate4dofMatrix(rx_IB_param,ry_IB_param,tx_IB_param,ty_IB_param);

T_BJ2_params = calibration_vec(5:8);
theta_BJ2_param = T_BJ2_params(1);
d_BJ2_param = T_BJ2_params(2);
a_BJ2_param = T_BJ2_params(3);
alpha_BJ2_param = T_BJ2_params(4);
T_BJ2_matrix = generateDHMatrix(theta_BJ2_param, d_BJ2_param, a_BJ2_param, alpha_BJ2_param);

T_J2J3_params = calibration_vec(9:12);
theta_J2J3_param = T_J2J3_params(1);
d_J2J3_param = T_J2J3_params(2);
a_J2J3_param = T_J2J3_params(3);
alpha_J2J3_param = T_J2J3_params(4);
T_J2J3_matrix = generateDHMatrix(theta_J2J3_param, d_J2J3_param, a_J2J3_param, alpha_J2J3_param);

T_J3D_params = calibration_vec(13:18);
theta_J3D_param = T_J3D_params(1);
d_J3D_param = T_J3D_params(2);
a_J3D_param = T_J3D_params(3);
alpha_J3D_param = T_J3D_params(4);
beta_J3D_param = T_J3D_params(5);
y_J3D_param = T_J3D_params(6);
T_J3D_matrix = generateModifiedDHMatrix(theta_J3D_param, d_J3D_param, a_J3D_param, alpha_J3D_param, beta_J3D_param, y_J3D_param);

R_IB = T_IB_matrix(1:3,1:3);
t_IB = T_IB_matrix(1:3,4);

T_IJ2_matrix = T_IB_matrix*T_BJ2_matrix;
R_IJ2 = T_IJ2_matrix(1:3,1:3);
t_IJ2 = T_IJ2_matrix(1:3,4);

T_IJ3_matrix = T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix;
R_IJ3 = T_IJ3_matrix(1:3,1:3);
t_IJ3 = T_IJ3_matrix(1:3,4);

T_ID_matrix = T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix;
R_ID = T_ID_matrix(1:3,1:3);
t_ID = T_ID_matrix(1:3,4);

T_BD_matrix = T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix;
R_BD = T_BD_matrix(1:3,1:3);
t_BD = T_BD_matrix(1:3,4);

T_J2D_matrix = T_J2J3_matrix*T_J3D_matrix;
R_J2D = T_J2D_matrix(1:3,1:3);
t_J2D = T_J2D_matrix(1:3,4);

R_J3D = T_J3D_matrix(1:3,1:3);
t_J3D = T_J3D_matrix(1:3,4);

t_ID_ss = skewSymmetricMatrix3(t_ID);
t_IB_ss = skewSymmetricMatrix3(t_IB);
t_BD_ss = skewSymmetricMatrix3(t_BD);
t_J2D_ss = skewSymmetricMatrix3(t_J2D);
t_J3D_ss = skewSymmetricMatrix3(t_J3D);
R_J3D_2_ss = skewSymmetricMatrix3(R_J3D(:,2));

T_IB_der_jac(:,1) = [1;0;0; -t_ID_ss(:,1)];
T_IB_der_jac(:,2) = [R_IB(:,2); -R_IB*t_BD_ss(:,2)-tx_IB_param*R_IB(:,3)];
T_IB_der_jac(:,3) = [0;0;0;R_IB(:,1)];
T_IB_der_jac(:,4) = [0;0;0;R_IB(:,2)]; 

T_BJ2_der_jac(:,1) = [R_IB(:,3); -R_IB*t_BD_ss(:,3)];
T_BJ2_der_jac(:,2) = [zeros(3,1); R_IB(:,3)];
T_BJ2_der_jac(:,3) = [zeros(3,1); R_IJ2(:,1)];
T_BJ2_der_jac(:,4) = [R_IJ2(:,1); -R_IJ2*t_J2D_ss(:,1)];

T_J2J3_der_jac(:,1) = [R_IJ2(:,3); -R_IJ2*t_J2D_ss(:,3)];
T_J2J3_der_jac(:,2) = [zeros(3,1); R_IJ2(:,3)];
T_J2J3_der_jac(:,3) = [zeros(3,1); R_IJ3(:,1)];
T_J2J3_der_jac(:,4) = [R_IJ3(:,1); -R_IJ3*t_J3D_ss(:,1)];

T_J3D_der_jac(:,1) = [R_IJ3(:,3); -R_IJ3*t_J3D_ss(:,3)];
T_J3D_der_jac(:,2) = [zeros(3,1); R_IJ3(:,3)];
T_J3D_der_jac(:,3) = [zeros(3,1); R_IJ3*R_J3D_2_ss(:,3)/cos(alpha_J3D_param)];
temp = R_IJ3*[y_J3D_param*sin(theta_J3D_param)*sin(alpha_J3D_param); -y_J3D_param*cos(theta_J3D_param)*sin(alpha_J3D_param); y_J3D_param*cos(alpha_J3D_param)];
T_J3D_der_jac(:,4) = [R_IJ3*R_J3D_2_ss(:,3)/cos(alpha_J3D_param); temp];
T_J3D_der_jac(:,5) = [R_ID(:,2); zeros(3,1)];
T_J3D_der_jac(:,6) = [zeros(3,1); R_ID(:,2)];

entire_chain_jacobian = [T_IB_der_jac T_BJ2_der_jac T_J2J3_der_jac T_J3D_der_jac];