function complete_jac = DHJacobians(theta, d, a, alpha)

%% Description
% Jacobian of the DH matrix wrt individual parameters

theta_jac = [0 0 1 -a*sin(theta) a*cos(theta) 0];

d_jac = [0 0 0 0 0 1];

a_jac = [0 0 0 cos(theta) sin(theta) 0];

alpha_jac = [cos(theta) sin(theta) 0 0 0 0];

complete_jac = [theta_jac' d_jac' a_jac' alpha_jac'];
