clear all
clc
close all

%% This file verifies the jacobians of the modified DH matrix wrt each parameter.

format long;

% Initializes random values
theta = rand;
d = rand;
a = rand;
alpha = rand;
beta = rand;
y = rand;

perturbation = sqrt(eps);

% Generates true and perturbed modified DH matrices
MDH_orig = generateModifiedDHMatrix(theta, d, a, alpha, beta, y);
MDH_theta_perturbed = generateModifiedDHMatrix(theta+perturbation, d, a, alpha, beta, y);
MDH_d_perturbed = generateModifiedDHMatrix(theta, d+perturbation, a, alpha, beta, y);
MDH_a_perturbed = generateModifiedDHMatrix(theta, d, a+perturbation, alpha, beta, y);
MDH_alpha_perturbed = generateModifiedDHMatrix(theta, d, a, alpha+perturbation, beta, y);
MDH_beta_perturbed = generateModifiedDHMatrix(theta, d, a, alpha, beta+perturbation, y);
MDH_y_perturbed = generateModifiedDHMatrix(theta, d, a, alpha, beta, y+perturbation);

% Gets modified DH jacobians
MDH_jac = MDHJacobians(theta, d, a, alpha, beta, y);

% Converts modified DH matrices to Transformation matrices
MDH_orig_T = Transformation(zeros(1,6));
MDH_theta_perturbed_T = Transformation(zeros(1,6));
MDH_d_perturbed_T = Transformation(zeros(1,6));
MDH_a_perturbed_T = Transformation(zeros(1,6));
MDH_alpha_perturbed_T = Transformation(zeros(1,6));
MDH_beta_perturbed_T = Transformation(zeros(1,6));
MDH_y_perturbed_T = Transformation(zeros(1,6));

MDH_orig_T.matrix = MDH_orig;
MDH_theta_perturbed_T.matrix = MDH_theta_perturbed;
MDH_d_perturbed_T.matrix = MDH_d_perturbed;
MDH_a_perturbed_T.matrix = MDH_a_perturbed;
MDH_alpha_perturbed_T.matrix = MDH_alpha_perturbed;
MDH_beta_perturbed_T.matrix = MDH_beta_perturbed;
MDH_y_perturbed_T.matrix = MDH_y_perturbed;

% Gets numerical jacobians of modified DH parameters
theta_jac_num = MDH_theta_perturbed_T.manifoldMinus(MDH_orig_T);
theta_jac_num = theta_jac_num./perturbation;
d_jac_num = MDH_d_perturbed_T.manifoldMinus(MDH_orig_T);
d_jac_num = d_jac_num./perturbation;
a_jac_num = MDH_a_perturbed_T.manifoldMinus(MDH_orig_T);
a_jac_num = a_jac_num./perturbation;
alpha_jac_num = MDH_alpha_perturbed_T.manifoldMinus(MDH_orig_T);
alpha_jac_num = alpha_jac_num./perturbation;
beta_jac_num = MDH_beta_perturbed_T.manifoldMinus(MDH_orig_T);
beta_jac_num = beta_jac_num./perturbation;
y_jac_num = MDH_y_perturbed_T.manifoldMinus(MDH_orig_T);
y_jac_num = y_jac_num./perturbation;

num_jac = [theta_jac_num d_jac_num a_jac_num alpha_jac_num beta_jac_num y_jac_num];

% Verifies analytical jacobian
for i=1:6
    norm_error = norm(num_jac(:,i)-MDH_jac(:,i));
    norm_num = norm(num_jac(:,i));
    norm_ana = norm(MDH_jac(:,i));
    error = norm_error/(norm_num+norm_ana);
    if error>1e-7
        disp(' test failed');
        result = 0;
    else
        disp(' test success');
        result = 1;
    end
end