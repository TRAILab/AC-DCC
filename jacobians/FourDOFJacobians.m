function jac = FourDOFJacobians(rx,ry,tx,ty)
%% Jacobian of the 4 DOF transform wrt each parameter.

jac_rx = [1 0 0 0 tx*sin(ry)*cos(rx)-ty*sin(rx) ty*cos(rx)+tx*sin(rx)*sin(ry)];

jac_ry = [0 cos(rx) sin(rx) -tx*sin(ry) tx*sin(rx)*cos(ry) -tx*cos(rx)*cos(ry)];

jac_tx = [0 0 0 cos(ry) sin(rx)*sin(ry) -cos(rx)*sin(ry)];

jac_ty = [0 0 0 0 cos(rx) sin(rx)];

jac = [jac_rx' jac_ry' jac_tx' jac_ty'];