clear all
clc
close all

%% Gets symbolic modified DH matrix

syms theta d a alpha beta y

T_theta = [cos(theta) -sin(theta) 0 0;
           sin(theta) cos(theta) 0 0;
           0 0 1 0;
           0 0 0 1];
T_d = [1 0 0 0;
       0 1 0 0;
       0 0 1 d;
       0 0 0 1];
T_a = [1 0 0 a;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
T_alpha = [1 0 0 0;
           0 cos(alpha) -sin(alpha) 0;
           0 sin(alpha) cos(alpha) 0;
           0 0 0 1];
T_beta = [cos(beta) 0 sin(beta) 0;
          0 1 0 0;
          -sin(beta) 0 cos(beta) 0;
          0 0 0 1];
T_y = [1 0 0 0;
       0 1 0 y;
       0 0 1 0;
       0 0 0 1];
   
T = T_theta*T_d*T_a*T_alpha*T_beta*T_y;