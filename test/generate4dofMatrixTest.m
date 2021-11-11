function M = generate4dofMatrixTest(rx,ry,tx,ty)

M = [cos(ry) 0 sin(ry) tx*cos(ry);
    sin(rx)*sin(ry) cos(rx) -sin(rx)*cos(ry) ty*cos(rx) + tx*sin(ry)*sin(rx);
    -sin(ry)*cos(rx) sin(rx) cos(rx)*cos(ry) ty*sin(rx) - tx*cos(rx)*sin(ry);
    0 0 0 1];