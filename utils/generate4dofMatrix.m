function M = generate4dofMatrix(rx,ry,tx,ty)

% This function generates a 4dof matrix used for the base to static camera
% transform
% M = [cos(ry) 0 sin(ry) tx*cos(ry);
%      sin(rx)*sin(ry) cos(rx) -sin(rx)*cos(ry) ty*cos(rx) + tx*sin(ry)*sin(rx);
%     -sin(ry)*cos(rx) sin(rx) cos(rx)*cos(ry) ty*sin(rx) - tx*cos(rx)*sin(ry);
%      0 0 0 1];

M = Transformation([rx, 0,  0, 0,  0,  0]).matrix*...
    Transformation([0,  ry, 0, 0,  0,  0]).matrix*...
    Transformation([0,  0,  0, tx, 0,  0]).matrix*...
    Transformation([0,  0,  0, 0,  ty, 0]).matrix;