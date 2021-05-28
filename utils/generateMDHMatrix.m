function [ M ] = generateMDHMatrix(theta,d,r,alpha,beta,ty)
%this function takes in the DH parameters and outputs the trans matrix
%theta: angle between X_i and X_{i+1}
%d: distance to common normal
%r: common normal length
%alpha angle Z_i to Z_{i+1}

% This transformation is from i to i-1
% Generate the inverse of the standard DH matrix:

M = [cos(theta)*cos(beta)-sin(theta)*sin(alpha)*sin(beta) -sin(theta)*cos(alpha)  cos(theta)*sin(beta)+sin(theta)*sin(alpha)*cos(beta) -ty*sin(theta)*cos(alpha)+r*cos(theta);
     sin(theta)*cos(beta)+cos(theta)*sin(alpha)*sin(beta)  cos(theta)*cos(alpha)  sin(theta)*sin(beta)-cos(theta)*sin(alpha)*cos(beta)  ty*cos(theta)*cos(alpha)+r*sin(theta);
     -cos(alpha)*sin(beta)                                 sin(alpha)             cos(alpha)*cos(beta)                                  ty*sin(alpha)+d;
     0                                                      0                      0                                                    1];

end