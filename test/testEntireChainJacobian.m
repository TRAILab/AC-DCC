clear all
clc
close all

%% Verifies the jacobians of the entire DCC chain.

format long;
perturbation = sqrt(eps);

% Generates values for minimal parameterization
chain_params = rand(1,18);

% Generates true and perturbed 4 DOF matrices
T_SB_params = chain_params(1:4);

rx_SB_param = T_SB_params(1);
ry_SB_param = T_SB_params(2);
tx_SB_param = T_SB_params(3);
ty_SB_param = T_SB_params(4);
T_SB_matrix = generate4dofMatrix(rx_SB_param, ry_SB_param, tx_SB_param, ty_SB_param);

rx_SB_param_pertubed = T_SB_params(1) + perturbation;
ry_SB_param_pertubed = T_SB_params(2) + perturbation;
tx_SB_param_pertubed = T_SB_params(3) + perturbation;
ty_SB_param_pertubed = T_SB_params(4) + perturbation;
T_SB_matrix_rx_perturbed = generate4dofMatrix(rx_SB_param_pertubed, ry_SB_param, tx_SB_param, ty_SB_param);
T_SB_matrix_ry_perturbed = generate4dofMatrix(rx_SB_param, ry_SB_param_pertubed, tx_SB_param, ty_SB_param);
T_SB_matrix_tx_perturbed = generate4dofMatrix(rx_SB_param, ry_SB_param, tx_SB_param_pertubed, ty_SB_param);
T_SB_matrix_ty_perturbed = generate4dofMatrix(rx_SB_param, ry_SB_param, tx_SB_param, ty_SB_param_pertubed);

% Generates DH matrix
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

% Generates DH matrix
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

% Generates Modified DH matrix
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

% Generates the entire transformation from dynamic to static camera
T_SD_matrix = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix);

% Generates perturbed transformation matrices
T_SD_matrix_rx_perturbed = Transformation(T_SB_matrix_rx_perturbed*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix);
T_SD_matrix_ry_perturbed = Transformation(T_SB_matrix_ry_perturbed*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix);
T_SD_matrix_tx_perturbed = Transformation(T_SB_matrix_tx_perturbed*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix);
T_SD_matrix_ty_perturbed = Transformation(T_SB_matrix_ty_perturbed*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix);

T_SD_matrix_BJ2_theta_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix_theta_perturbed*T_J2J3_matrix*T_J3D_matrix);
T_SD_matrix_BJ2_d_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix_d_perturbed*T_J2J3_matrix*T_J3D_matrix);
T_SD_matrix_BJ2_a_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix_a_perturbed*T_J2J3_matrix*T_J3D_matrix);
T_SD_matrix_BJ2_alpha_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix_alpha_perturbed*T_J2J3_matrix*T_J3D_matrix);

T_SD_matrix_J2J3_theta_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix_theta_perturbed*T_J3D_matrix);
T_SD_matrix_J2J3_d_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix_d_perturbed*T_J3D_matrix);
T_SD_matrix_J2J3_a_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix_a_perturbed*T_J3D_matrix);
T_SD_matrix_J2J3_alpha_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix_alpha_perturbed*T_J3D_matrix);

T_SD_matrix_J3D_theta_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_theta_perturbed);
T_SD_matrix_J3D_d_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_d_perturbed);
T_SD_matrix_J3D_a_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_a_perturbed);
T_SD_matrix_J3D_alpha_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_alpha_perturbed);
T_SD_matrix_J3D_beta_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_beta_perturbed);
T_SD_matrix_J3D_y_perturbed = Transformation(T_SB_matrix*T_BJ2_matrix*T_J2J3_matrix*T_J3D_matrix_y_perturbed);

% Gets numerical jacobians
T_SD_num_jac = zeros(6,18);
T_SD_num_jac(:,1) = T_SD_matrix_rx_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,2) = T_SD_matrix_ry_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,3) = T_SD_matrix_tx_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,4) = T_SD_matrix_ty_perturbed.manifoldMinus(T_SD_matrix);

T_SD_num_jac(:,5) = T_SD_matrix_BJ2_theta_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,6) = T_SD_matrix_BJ2_d_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,7) = T_SD_matrix_BJ2_a_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,8) = T_SD_matrix_BJ2_alpha_perturbed.manifoldMinus(T_SD_matrix);

T_SD_num_jac(:,9) = T_SD_matrix_J2J3_theta_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,10) = T_SD_matrix_J2J3_d_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,11) = T_SD_matrix_J2J3_a_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,12) = T_SD_matrix_J2J3_alpha_perturbed.manifoldMinus(T_SD_matrix);

T_SD_num_jac(:,13) = T_SD_matrix_J3D_theta_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,14) = T_SD_matrix_J3D_d_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,15) = T_SD_matrix_J3D_a_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,16) = T_SD_matrix_J3D_alpha_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,17) = T_SD_matrix_J3D_beta_perturbed.manifoldMinus(T_SD_matrix);
T_SD_num_jac(:,18) = T_SD_matrix_J3D_y_perturbed.manifoldMinus(T_SD_matrix);

% Final numerical jacobian
T_SD_num_jac = T_SD_num_jac./perturbation;

% Gets analytical jacobian
T_SB_matrix = Transformation(T_SB_matrix);
T_BJ2_matrix = Transformation(T_BJ2_matrix);
T_J2J3_matrix = Transformation(T_J2J3_matrix);
T_J3D_matrix = Transformation(T_J3D_matrix);
T_BD_matrix = Transformation(T_BJ2_matrix.matrix*T_J2J3_matrix.matrix*T_J3D_matrix.matrix);
T_SJ2_matrix = Transformation(T_SB_matrix.matrix*T_BJ2_matrix.matrix);
T_J2D_matrix = Transformation(T_J2J3_matrix.matrix*T_J3D_matrix.matrix);
T_SJ3_matrix = Transformation(T_SB_matrix.matrix*T_BJ2_matrix.matrix*T_J2J3_matrix.matrix);

T_SD_ana_jac = zeros(6,18);

% Gets analytical jacobian of T_SB
[~, T_SD_SB_jac, ~] = T_SB_matrix.composeAndJacobian(T_BD_matrix);
IB_jac = FourDOFJacobians(rx_SB_param,ry_SB_param,tx_SB_param,ty_SB_param);
T_SD_ana_jac(:, 1:4) = T_SD_SB_jac*IB_jac;

% Gets analytical jacobian of T_BJ2
[~, T_SD_SJ2, ~] = T_SJ2_matrix.composeAndJacobian(T_J2D_matrix);
[~, ~, T_SJ2_BJ2] = T_SB_matrix.composeAndJacobian(T_BJ2_matrix);
BJ2_jacobians = DHJacobians(theta_BJ2_param, d_BJ2_param, a_BJ2_param, alpha_BJ2_param);
T_SD_ana_jac(:, 5:8) = T_SD_SJ2*T_SJ2_BJ2*BJ2_jacobians;

% Gets analytical jacobian of T_J2J3
[~, ~, T_SD_J2D] = T_SJ2_matrix.composeAndJacobian(T_J2D_matrix);
[~, T_J2D_J2J3, ~] = T_J2J3_matrix.composeAndJacobian(T_J3D_matrix);
J2J3_jacobians = DHJacobians(theta_J2J3_param, d_J2J3_param, a_J2J3_param, alpha_J2J3_param);
T_SD_ana_jac(:, 9:12) = T_SD_J2D*T_J2D_J2J3*J2J3_jacobians;

% Gets analytical jacobian of T_J3D
[~, ~, T_SD_J3D] = T_SJ3_matrix.composeAndJacobian(T_J3D_matrix);
J3D_jacobians = MDHJacobians(theta_J3D_param, d_J3D_param, a_J3D_param, alpha_J3D_param, beta_J3D_param, y_J3D_param);
T_SD_ana_jac(:, 13:18) = T_SD_J3D*J3D_jacobians;

% Verifies analytical jacobian
for i=1:18
    norm_error = norm(T_SD_ana_jac(:,i) - T_SD_num_jac(:,i));
    norm_ana = norm(T_SD_ana_jac(:,i));
    norm_num = norm(T_SD_num_jac(:,i));
    error = norm_error/(norm_num+norm_ana);
    if error>1e-7
        disp(' test failed');
        result = 0;
    else
        disp(' test success');
        result = 1;
    end
end

T_SD_ana_jac;

%% Analytical jacobians based on derivations

T_SD_der_jac_new = getEntireChainJacobian(chain_params);

diff_ana_and_der = T_SD_ana_jac(:,1:18) - T_SD_der_jac_new(:,1:18);
if isempty(find(diff_ana_and_der > 1e-12))
    disp('complete jacobian is a success');
else
    disp('sorry it failed');
end