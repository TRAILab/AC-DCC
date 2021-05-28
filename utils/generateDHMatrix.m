function [ M ] = generateStandardDHMmatrix( theta,d,r,alpha)
%this function takes in the DH parameters and outputs the trans matrix
%theta: angle between X_i and X_{i+1}
%d: distance to common normal
%r: common normal length
%alpha angle Z_i to Z_{i+1}

% This transformation is from i to i-1
% Generate the inverse of the standard DH matrix:

M = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) r*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) r*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];

end

