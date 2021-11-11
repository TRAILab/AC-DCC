clear all
clc
close all

% This script is used to test the entire jacobian chain. it T_ID/each of
% the 18 parameters

format long;
perturbation = sqrt(eps);

% Generate the 18 parameters
chain_params = rand(1,18);

%%
T_IB_params = chain_params(1:4);

rx_IB_param = T_IB_params(1);
ry_IB_param = T_IB_params(2);
tx_IB_param = T_IB_params(3);
ty_IB_param = T_IB_params(4);
T_IB_matrix = generate4dofMatrix(rx_IB_param,ry_IB_param,tx_IB_param,ty_IB_param);

rx_IB_param_pertubed = T_IB_params(1) + perturbation;
ry_IB_param_pertubed = T_IB_params(2) + perturbation;
tx_IB_param_pertubed = T_IB_params(3) + perturbation;
ty_IB_param_pertubed = T_IB_params(4) + perturbation;
T_IB_matrix_rx_perturbed = generate4dofMatrix(rx_IB_param_pertubed, ry_IB_param, tx_IB_param, ty_IB_param);
T_IB_matrix_ry_perturbed = generate4dofMatrix(rx_IB_param, ry_IB_param_pertubed, tx_IB_param, ty_IB_param);
T_IB_matrix_tx_perturbed = generate4dofMatrix(rx_IB_param, ry_IB_param, tx_IB_param_pertubed, ty_IB_param);
T_IB_matrix_ty_perturbed = generate4dofMatrix(rx_IB_param, ry_IB_param, tx_IB_param, ty_IB_param_pertubed);

%%
T_BJ2_params = chain_params(5:8);

theta_BJ2_param = T_BJ2_params(1);
d_BJ2_param = T_BJ2_params(2);
a_BJ2_param = T_BJ2_params(3);
alpha_BJ2_param = T_BJ2_params(4);
T_BJ2_matrix = generateDHMatrix(theta_BJ2_param, d_BJ2_param, a_BJ2_param, alpha_BJ2_param);

theta_BJ2_param_pertubed = T_BJ2_params(1) + perturbation;
d_BJ2_param_pertubed = T_BJ2_params(2) + perturbation;
a_BJ2_param_pertubed = T_BJ2_params(3) + perturbation;
alpha_BJ2_param_pertubed = T_BJ2_params(4) + perturbation;
T_BJ2_matrix_theta_perturbed = generateDHMatrix(theta_BJ2_param_pertubed, d_BJ2_param, a_BJ2_param, alpha_BJ2_param);
T_BJ2_matrix_d_perturbed = generateDHMatrix(theta_BJ2_param, d_BJ2_param_pertubed, a_BJ2_param, alpha_BJ2_param);
T_BJ2_matrix_a_perturbed = generateDHMatrix(theta_BJ2_param, d_BJ2_param, a_BJ2_param_pertubed, alpha_BJ2_param);
T_BJ2_matrix_alpha_perturbed = generateDHMatrix(theta_BJ2_param, d_BJ2_param, a_BJ2_param, alpha_BJ2_param_pertubed);

%%
T_J2J3_params = chain_params(9:12);

theta_J2J3_param = T_J2J3_params(1);
d_J2J3_param = T_J2J3_params(2);
a_J2J3_param = T_J2J3_params(3);
alpha_J2J3_param = T_J2J3_params(4);
T_J2J3_matrix = generateDHMatrix(theta_J2J3_param, d_J2J3_param, a_J2J3_param, alpha_J2J3_param);

theta_J2J3_param_pertubed = T_J2J3_params(1) + perturbation;
d_J2J3_param_pertubed = T_J2J3_params(2) + perturbation;
a_J2J3_param_pertubed = T_J2J3_params(3) + perturbation;
alpha_J2J3_param_pertubed = T_J2J3_params(4) + perturbation;
T_J2J3_matrix_theta_perturbed = generateDHMatrix(theta_J2J3_param_pertubed, d_J2J3_param, a_J2J3_param, alpha_J2J3_param);
T_J2J3_matrix_d_perturbed = generateDHMatrix(theta_J2J3_param, d_J2J3_param_pertubed, a_J2J3_param, alpha_J2J3_param);
T_J2J3_matrix_a_perturbed = generateDHMatrix(theta_J2J3_param, d_J2J3_param, a_J2J3_param_pertubed, alpha_J2J3_param);
T_J2J3_matrix_alpha_perturbed = generateDHMatrix(theta_J2J3_param, d_J2J3_param, a_J2J3_param, alpha_J2J3_param_pertubed);

%%
T_J3D_params = chain_params(13:18);

theta_J3D_param = T_J3D_params(1);
d_J3D_param = T_J3D_params(2);
a_J3D_param = T_J3D_params(3);
alpha_J3D_param = T_J3D_params(4);
beta_J3D_param = T_J3D_params(5);
y_J3D_param = T_J3D_params(6);
T_J3D_matrix = generateModifiedDHMatrix(theta_J3D_param, d_J3D_param, a_J3D_param, alpha_J3D_param, beta_J3D_param, y_J3D_param);

theta_J3D_param_perturbed = T_J3D_params(1) + perturbation;
d_J3D_param_perturbed = T_J3D_params(2) + perturbation;
a_J3D_param_perturbed = T_J3D_params(3) + perturbation;
alpha_J3D_param_perturbed = T_J3D_params(4) + perturbation;
beta_J3D_param_perturbed = T_J3D_params(5) + perturbation;
y_J3D_param_perturbed = T_J3D_params(6) + perturbation;
T_J3D_matrix_theta_perturbed = generateModifiedDHMatrix(theta_J3D_param_perturbed, d_J3D_param, a_J3D_param, alpha_J3D_param, beta_J3D_param, y_J3D_param);
T_J3D_matrix_d_perturbed = generateModifiedDHMatrix(theta_J3D_param, d_J3D_param_perturbed, a_J3D_param, alpha_J3D_param, beta_J3D_param, y_J3D_param);
T_J3D_matrix_a_perturbed = generateModifiedDHMatrix(theta_J3D_param, d_J3D_param, a_J3D_param_perturbed, alpha_J3D_param, beta_J3D_param, y_J3D_param);
T_J3D_matrix_alpha_perturbed = generateModifiedDHMatrix(theta_J3D_param, d_J3D_param, a_J3D_param, alpha_J3D_param_perturbed, beta_J3D_param, y_J3D_param);
T_J3D_matrix_beta_perturbed = generateModifiedDHMatrix(theta_J3D_param, d_J3D_param, a_J3D_param, alpha_J3D_param, beta_J3D_param_perturbed, y_J3D_param);
T_J3D_matrix_y_perturbed = generateModifiedDHMatrix(theta_J3D_param, d_J3D_param, a_J3D_param, alpha_J3D_param, beta_J3D_param, y_J3D_param_perturbed);

%%
T_ID_matrix = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix);

%% 
T_ID_matrix_rx_perturbed = Transformation(T_IB_matrix_rx_perturbed*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix);
T_ID_matrix_ry_perturbed = Transformation(T_IB_matrix_ry_perturbed*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix);
T_ID_matrix_tx_perturbed = Transformation(T_IB_matrix_tx_perturbed*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix);
T_ID_matrix_ty_perturbed = Transformation(T_IB_matrix_ty_perturbed*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix);

T_ID_matrix_BJ2_theta_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix_theta_perturbed*T_J2J3_matrix*T_J3D_matrix);
T_ID_matrix_BJ2_d_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix_d_perturbed*T_J2J3_matrix*T_J3D_matrix);
T_ID_matrix_BJ2_a_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix_a_perturbed*T_J2J3_matrix*T_J3D_matrix);
T_ID_matrix_BJ2_alpha_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix_alpha_perturbed*T_J2J3_matrix*T_J3D_matrix);

T_ID_matrix_J2J3_theta_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix_theta_perturbed*T_J3D_matrix);
T_ID_matrix_J2J3_d_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix_d_perturbed*T_J3D_matrix);
T_ID_matrix_J2J3_a_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix_a_perturbed*T_J3D_matrix);
T_ID_matrix_J2J3_alpha_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix_alpha_perturbed*T_J3D_matrix);

T_ID_matrix_J3D_theta_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_theta_perturbed);
T_ID_matrix_J3D_d_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_d_perturbed);
T_ID_matrix_J3D_a_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_a_perturbed);
T_ID_matrix_J3D_alpha_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_alpha_perturbed);
T_ID_matrix_J3D_beta_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_beta_perturbed);
T_ID_matrix_J3D_y_perturbed = Transformation(T_IB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_y_perturbed);

T_ID_num_jac = zeros(6,18);
T_ID_num_jac(:,1) = T_ID_matrix_rx_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,2) = T_ID_matrix_ry_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,3) = T_ID_matrix_tx_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,4) = T_ID_matrix_ty_perturbed.manifoldMinus(T_ID_matrix);

T_ID_num_jac(:,5) = T_ID_matrix_BJ2_theta_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,6) = T_ID_matrix_BJ2_d_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,7) = T_ID_matrix_BJ2_a_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,8) = T_ID_matrix_BJ2_alpha_perturbed.manifoldMinus(T_ID_matrix);

T_ID_num_jac(:,9) = T_ID_matrix_J2J3_theta_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,10) = T_ID_matrix_J2J3_d_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,11) = T_ID_matrix_J2J3_a_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,12) = T_ID_matrix_J2J3_alpha_perturbed.manifoldMinus(T_ID_matrix);

T_ID_num_jac(:,13) = T_ID_matrix_J3D_theta_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,14) = T_ID_matrix_J3D_d_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,15) = T_ID_matrix_J3D_a_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,16) = T_ID_matrix_J3D_alpha_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,17) = T_ID_matrix_J3D_beta_perturbed.manifoldMinus(T_ID_matrix);
T_ID_num_jac(:,18) = T_ID_matrix_J3D_y_perturbed.manifoldMinus(T_ID_matrix);

%% Final numerical jacobian
T_ID_num_jac = T_ID_num_jac./perturbation;

%% Calcualte analytical jacobian
T_IB_matrix = Transformation(T_IB_matrix);
T_BJ2_matrix = Transformation(T_BJ2_matrix);
T_J2J3_matrix = Transformation(T_J2J3_matrix);
T_J3D_matrix = Transformation(T_J3D_matrix);
T_BD_matrix = Transformation(T_BJ2_matrix.matrix*T_J2J3_matrix.matrix*T_J3D_matrix.matrix);
T_IJ2_matrix = Transformation(T_IB_matrix.matrix*T_BJ2_matrix.matrix);
T_IJ2_matrix = Transformation(T_IB_matrix.matrix*T_BJ2_matrix.matrix);
T_J2D_matrix = Transformation(T_J2J3_matrix.matrix*T_J3D_matrix.matrix);
T_IJ3_matrix = Transformation(T_IB_matrix.matrix*T_BJ2_matrix.matrix*T_J2J3_matrix.matrix);

T_ID_ana_jac = zeros(6,18);

% IB
[~, T_ID_IB_jac, ~] = T_IB_matrix.composeAndJacobian(T_BD_matrix);
IB_jac = FourDOFJacobians(rx_IB_param,ry_IB_param,tx_IB_param,ty_IB_param);
T_ID_ana_jac(:,1:4) = T_ID_IB_jac*IB_jac;

% BJ2
[~, T_ID_IJ2, ~] = T_IJ2_matrix.composeAndJacobian(T_J2D_matrix);
[~, ~, T_IJ2_BJ2] = T_IB_matrix.composeAndJacobian(T_BJ2_matrix);
BJ2_jacobians = DHJacobians(theta_BJ2_param, d_BJ2_param, a_BJ2_param, alpha_BJ2_param);
T_ID_ana_jac(:,5:8) = T_ID_IJ2*T_IJ2_BJ2*BJ2_jacobians;

% J2J3
[~, ~, T_ID_J2D] = T_IJ2_matrix.composeAndJacobian(T_J2D_matrix);
[~, T_J2D_J2J3, ~] = T_J2J3_matrix.composeAndJacobian(T_J3D_matrix);
J2J3_jacobians = DHJacobians(theta_J2J3_param, d_J2J3_param, a_J2J3_param, alpha_J2J3_param);
T_ID_ana_jac(:,9:12) = T_ID_J2D*T_J2D_J2J3*J2J3_jacobians;

% J3D
[~, ~, T_ID_J3D] = T_IJ3_matrix.composeAndJacobian(T_J3D_matrix);
J3D_jacobians = MDHJacobians(theta_J3D_param, d_J3D_param, a_J3D_param, alpha_J3D_param, beta_J3D_param, y_J3D_param);
T_ID_ana_jac(:,13:18) = T_ID_J3D*J3D_jacobians;

for i=1:18
    norm_error = norm(T_ID_ana_jac(:,i)-T_ID_num_jac(:,i));
    norm_ana = norm(T_ID_ana_jac(:,i));
    norm_num = norm(T_ID_num_jac(:,i));
    error = norm_error/(norm_num+norm_ana);
    if error>1e-7
        disp(' test failed');
        result = 0;
    else
        disp(' test success');
        result = 1;
    end
end

T_ID_ana_jac;

%% Analytical jacobians based on derivations (see hand written pages)

T_ID_der_jac_new = getEntireChainJacobian(chain_params);

diff_ana_and_der = T_ID_ana_jac(:,1:18) - T_ID_der_jac_new(:,1:18);
if isempty(find(diff_ana_and_der > 1e-12))
    disp('its a success');
else
    disp('sorry it failed');
end
% 
% T_ID_der_jac = zeros(6,18);
% 
% R_IB = T_IB_matrix.matrix(1:3,1:3);
% t_IB = T_IB_matrix.matrix(1:3,4);
% 
% R_IJ2 = T_IJ2_matrix.matrix(1:3,1:3);
% t_IJ2 = T_IJ2_matrix.matrix(1:3,4);
% 
% R_IJ3 = T_IJ3_matrix.matrix(1:3,1:3);
% t_IJ3 = T_IJ3_matrix.matrix(1:3,4);
% 
% R_ID = T_ID_matrix.matrix(1:3,1:3);
% t_ID = T_ID_matrix.matrix(1:3,4);
% 
% R_BD = T_BD_matrix.matrix(1:3,1:3);
% t_BD = T_BD_matrix.matrix(1:3,4);
% 
% R_J2D = T_J2D_matrix.matrix(1:3,1:3);
% t_J2D = T_J2D_matrix.matrix(1:3,4);
% 
% R_J3D = T_J3D_matrix.matrix(1:3,1:3);
% t_J3D = T_J3D_matrix.matrix(1:3,4);
% 
% t_ID_ss = skewSymmetricMatrix3(t_ID);
% t_IB_ss = skewSymmetricMatrix3(t_IB);
% t_BD_ss = skewSymmetricMatrix3(t_BD);
% t_J2D_ss = skewSymmetricMatrix3(t_J2D);
% t_J3D_ss = skewSymmetricMatrix3(t_J3D);
% R_J3D_2_ss = skewSymmetricMatrix3(R_J3D(:,2));
% 
% T_IB_der_jac(:,1) = [1;0;0; -t_ID_ss(:,1)];
% T_IB_der_jac(:,2) = [R_IB(:,2); -R_IB*t_BD_ss(:,2)-tx_IB_param*R_IB(:,3)];
% T_IB_der_jac(:,3) = [0;0;0;R_IB(:,1)];
% T_IB_der_jac(:,4) = [0;0;0;R_IB(:,2)]; 
% 
% T_BJ2_der_jac(:,1) = [R_IB(:,3); -R_IB*t_BD_ss(:,3)];
% T_BJ2_der_jac(:,2) = [zeros(3,1); R_IB(:,3)];
% T_BJ2_der_jac(:,3) = [zeros(3,1); R_IJ2(:,1)];
% T_BJ2_der_jac(:,4) = [R_IJ2(:,1); -R_IJ2*t_J2D_ss(:,1)];
% 
% T_J2J3_der_jac(:,1) = [R_IJ2(:,3); -R_IJ2*t_J2D_ss(:,3)];
% T_J2J3_der_jac(:,2) = [zeros(3,1); R_IJ2(:,3)];
% T_J2J3_der_jac(:,3) = [zeros(3,1); R_IJ3(:,1)];
% T_J2J3_der_jac(:,4) = [R_IJ3(:,1); -R_IJ3*t_J3D_ss(:,1)];
% 
% T_J3D_der_jac(:,1) = [R_IJ3(:,3); -R_IJ3*t_J3D_ss(:,3)];
% T_J3D_der_jac(:,2) = [zeros(3,1); R_IJ3(:,3)];
% T_J3D_der_jac(:,3) = [zeros(3,1); R_IJ3*R_J3D_2_ss(:,3)/cos(alpha_J3D_param)];
% temp = R_IJ3*[y_J3D_param*sin(theta_J3D_param)*sin(alpha_J3D_param); -y_J3D_param*cos(theta_J3D_param)*sin(alpha_J3D_param); y_J3D_param*cos(alpha_J3D_param)];
% T_J3D_der_jac(:,4) = [R_IJ3*R_J3D_2_ss(:,3)/cos(alpha_J3D_param); temp];
% T_J3D_der_jac(:,5) = [R_ID(:,2); zeros(3,1)];
% T_J3D_der_jac(:,6) = [zeros(3,1); R_ID(:,2)];
% 
% temp1(:,1) = [0;0;1;-t_J3D_ss(:,3)];
% temp1(:,2) = [0;0;0;0;0;1];
% temp1(:,3) = [0;0;0;R_J3D_2_ss(:,3)/cos(alpha_J3D_param)];
% temp1(:,4) = [R_J3D_2_ss(:,3)/cos(alpha_J3D_param); y_J3D_param*sin(theta_J3D_param)*sin(alpha_J3D_param); -y_J3D_param*cos(theta_J3D_param)*sin(alpha_J3D_param); y_J3D_param*cos(alpha_J3D_param)];
% temp1(:,5) = [R_J3D(:,2);0;0;0];
% temp1(:,6) = [0;0;0;R_J3D(:,2)];
% temp2 = zeros(6,6);
% temp2(1:3,1:3) = R_IJ3;
% temp2(4:6,4:6) = R_IJ3;
% temp3 = temp2*temp1;
% 
% T_ID_der_jac(:,1:18) = [T_IB_der_jac T_BJ2_der_jac T_J2J3_der_jac T_J3D_der_jac];
