function [ M ] = generateModifiedDHMatrix(theta, d, r, alpha, beta, ty)

%% Description
% This is a 6-DOF modified DH matrix
% This is the transformation from i to i-1

M = [cos(theta)*cos(beta)-sin(theta)*sin(alpha)*sin(beta) -sin(theta)*cos(alpha)  cos(theta)*sin(beta)+sin(theta)*sin(alpha)*cos(beta) -ty*sin(theta)*cos(alpha)+r*cos(theta);
     sin(theta)*cos(beta)+cos(theta)*sin(alpha)*sin(beta)  cos(theta)*cos(alpha)  sin(theta)*sin(beta)-cos(theta)*sin(alpha)*cos(beta)  ty*cos(theta)*cos(alpha)+r*sin(theta);
     -cos(alpha)*sin(beta)                                 sin(alpha)             cos(alpha)*cos(beta)                                  ty*sin(alpha)+d;
     0                                                      0                      0                                                    1];

end