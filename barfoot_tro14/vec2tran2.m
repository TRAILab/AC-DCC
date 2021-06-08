function [T] = vec2tran2(p)

validateattributes(p,{'double'},{'size',[6,1]});

phi = p(1:3);
rho = p(4:6);

C = vec2rot(phi);

T = eye(4);
T(1:3,1:3) = C;
T(1:3,4) = rho;