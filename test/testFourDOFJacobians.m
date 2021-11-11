clear all
clc
close all

format long;

rx = rand;
ry = rand;
tx = rand;
ty = rand;

perturbation = sqrt(eps);

orig_4dof =         generate4dofMatrix(rx, ry, tx, ty);
rx_4dof_perturbed = generate4dofMatrix(rx+perturbation, ry, tx, ty);
ry_4dof_perturbed = generate4dofMatrix(rx, ry+perturbation, tx, ty);
tx_4dof_perturbed = generate4dofMatrix(rx, ry, tx+perturbation, ty);
ty_4dof_perturbed = generate4dofMatrix(rx, ry, tx, ty+perturbation);

ana_jac = FourDOFJacobians(rx,ry,tx,ty);

orig_4dof_T = Transformation(zeros(1,6));
rx_4dof_perturbed_T = Transformation(zeros(1,6));
ry_4dof_perturbed_T = Transformation(zeros(1,6));
tx_4dof_perturbed_T = Transformation(zeros(1,6));
ty_4dof_perturbed_T = Transformation(zeros(1,6));

orig_4dof_T.matrix = orig_4dof;
rx_4dof_perturbed_T.matrix = rx_4dof_perturbed;
ry_4dof_perturbed_T.matrix = ry_4dof_perturbed;
tx_4dof_perturbed_T.matrix = tx_4dof_perturbed;
ty_4dof_perturbed_T.matrix = ty_4dof_perturbed;

rx_jac_num = rx_4dof_perturbed_T.manifoldMinus(orig_4dof_T);
rx_jac_num = rx_jac_num./perturbation;
ry_jac_num = ry_4dof_perturbed_T.manifoldMinus(orig_4dof_T);
ry_jac_num = ry_jac_num./perturbation;
tx_jac_num = tx_4dof_perturbed_T.manifoldMinus(orig_4dof_T);
tx_jac_num = tx_jac_num./perturbation;
ty_jac_num = ty_4dof_perturbed_T.manifoldMinus(orig_4dof_T);
ty_jac_num = ty_jac_num./perturbation;

num_jac = [rx_jac_num ry_jac_num tx_jac_num ty_jac_num];

for i=1:4
    norm_error = norm(num_jac(:,i)-ana_jac(:,i));
    norm_num = norm(num_jac(:,i));
    norm_ana = norm(ana_jac(:,i));
    error = norm_error/(norm_num+norm_ana);
    if error>1e-6
        disp(' test failed');
        result = 0;
    else
        disp(' test success');
        result = 1;
    end
end