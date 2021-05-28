function M = generateFourDOFMatrix(rx,ry,tx,ty)

% This function generates a 4dof matrix used for the base to static camera
% transform

M = Transformation([rx, 0,  0, 0,  0,  0]).matrix*...
    Transformation([0,  ry, 0, 0,  0,  0]).matrix*...
    Transformation([0,  0,  0, tx, 0,  0]).matrix*...
    Transformation([0,  0,  0, 0,  ty, 0]).matrix;
    

