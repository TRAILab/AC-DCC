clear all
clc
close all

%% Gets symbolic 4 DOF matrix

syms rx ry tx ty

T_tx = [1 0 0 tx;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
T_rx = [1 0 0 0;
           0 cos(rx) -sin(rx) 0;
           0 sin(rx) cos(rx) 0;
           0 0 0 1];
T_ry = [cos(ry) 0 sin(ry) 0;
          0 1 0 0;
          -sin(ry) 0 cos(ry) 0;
          0 0 0 1];
T_ty = [1 0 0 0;
       0 1 0 ty;
       0 0 1 0;
       0 0 0 1];
   
T = T_rx*T_ry*T_tx*T_ty;