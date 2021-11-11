clear all
clc
close all

format long;

theta = rand;
d = rand;
a = rand;
alpha = rand;

perturbation = sqrt(eps);

DH_orig = generateDHMatrix(theta, d, a, alpha);
DH_theta_perturbed = generateDHMatrix(theta+perturbation, d, a, alpha);
DH_d_perturbed = generateDHMatrix(theta, d+perturbation, a, alpha);
DH_a_perturbed = generateDHMatrix(theta, d, a+perturbation, alpha);
DH_alpha_perturbed = generateDHMatrix(theta, d, a, alpha+perturbation);

DH_jac = DHJacobians(theta, d, a, alpha);

DH_orig_T = Transformation(zeros(1,6));
DH_theta_perturbed_T = Transformation(zeros(1,6));
DH_d_perturbed_T = Transformation(zeros(1,6));
DH_a_perturbed_T = Transformation(zeros(1,6));
DH_alpha_perturbed_T = Transformation(zeros(1,6));

DH_orig_T.matrix = DH_orig;
DH_theta_perturbed_T.matrix = DH_theta_perturbed;
DH_d_perturbed_T.matrix = DH_d_perturbed;
DH_a_perturbed_T.matrix = DH_a_perturbed;
DH_alpha_perturbed_T.matrix = DH_alpha_perturbed;

theta_jac_num = DH_theta_perturbed_T.manifoldMinus(DH_orig_T);
theta_jac_num = theta_jac_num./perturbation;
d_jac_num = DH_d_perturbed_T.manifoldMinus(DH_orig_T);
d_jac_num = d_jac_num./perturbation;
a_jac_num = DH_a_perturbed_T.manifoldMinus(DH_orig_T);
a_jac_num = a_jac_num./perturbation;
alpha_jac_num = DH_alpha_perturbed_T.manifoldMinus(DH_orig_T);
alpha_jac_num = alpha_jac_num./perturbation;

num_jac = [theta_jac_num d_jac_num a_jac_num alpha_jac_num];

for i=1:4
    norm_error = norm(num_jac(:,i)-DH_jac(:,i));
    norm_num = norm(num_jac(:,i));
    norm_ana = norm(DH_jac(:,i));
    error = norm_error/(norm_num+norm_ana);
    if error>1e-7
        disp(' test failed');
        result = 0;
    else
        disp(' test success');
        result = 1;
    end
end