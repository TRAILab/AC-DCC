clear all
clc
close all

% This file generates the plot for the sensitivity analysis experiment

static1_err = [0.3127 1.5085 3.3466;
               0.3079 1.6689 3.2046;
               0.3155 1.5388 3.1359];

static2_err = [0.3035 1.6215 3.2847;
               0.2840 1.4701 2.8855;
               0.2956 1.4223 3.3151];
            
[X1,Y1] = meshgrid(1:size(static1_err,2), 1:size(static1_err,1));
X1 = X1(:);
Y1 = Y1(:);

X = [0.2;0.2;0.2;1;1;1;2;2;2];
Y = [0.1;7;10;0.1;7;10;0.1;7;10];

figure(1);
hb1 = bar3(static1_err);
set(gca,'XTickLabel',[0.2 1 2],'FontSize', 20);
set(gca,'YTickLabel',[10 7 0.1],'FontSize', 20);
text(X1, Y1, static1_err(:), num2str(static1_err(:)), 'HorizontalAlignment','center', 'VerticalAlignment','bottom','FontSize', 20);
title('Static 1 Camera','FontSize', 20);
xlabel('Pixel Noise','FontSize', 20);
ylabel('Encoder Noise','FontSize', 20);
zlabel('Pixel Re-projection Error','FontSize', 20);
hb1(1).FaceColor = '#D95319';

figure(2);
hb2 = bar3(static2_err);
set(gca,'XTickLabel',[0.2 1 2],'FontSize', 20);
set(gca,'YTickLabel',[10 7 0.1],'FontSize', 20);
text(X1, Y1, static2_err(:), num2str(static2_err(:)), 'HorizontalAlignment','center', 'VerticalAlignment','bottom','FontSize', 20);
title('Static 2 Camera','FontSize', 20);
xlabel('Pixel Noise','FontSize', 20);
ylabel('Encoder Noise','FontSize', 20);
zlabel('Pixel Re-projection Error','FontSize', 20);
hb2(1).FaceColor = '#D95319';

% Calculate mean and std deviation pixel error over all cameras and all
% experiments with same pixel noise
pn1_mean_train = [0.24588 0.25002 0.24818 0.24871 0.25036 0.24594 0.24181 0.24803 0.24743];
pn1_sd_train = [0.016036 0.010159 0.011892 0.014959 0.011454 0.0089446 0.013349 0.011292 0.011138];
pn2_mean_train = [1.2255 1.2552 1.2441 1.2085 1.2624 1.2492 1.2264 1.2515 1.2459];
pn2_sd_train = [0.090069 0.054572 0.059828 0.074089 0.065698 0.046327 0.07786 0.048137 0.05337];
pn3_mean_train = [2.4682 2.4786 2.4608 2.404 2.4719 2.4844 2.4157 2.4853 2.5068];
pn3_sd_train = [0.13878 0.095719 0.10007 0.14026 0.097111 0.10589 0.17152 0.096086 0.12524];

pn1_mean_test = [0.24195 0.24677 0.25009 0.24503 0.24539 0.24827 0.24427 0.24638 0.24844];
pn1_sd_test = [0.01354 0.010218 0.012759 0.015193 0.0087597 0.0082946 0.018445 0.009548 0.0081387];
pn2_mean_test = [1.1876 1.2391 1.2477 1.2268 1.2375 1.2448 1.1798 1.2307 1.2376];
pn2_sd_test = [0.077269 0.048835 0.04891 0.092649 0.050895 0.052008 0.08352 0.060264 0.046392];
pn3_mean_test = [2.4602 2.4732 2.2882 2.4622 2.4744 2.5345 2.4667 2.4855 2.4768];
pn3_sd_test = [0.19995 0.096438 0.10107 0.16803 0.093519 0.082031 0.13331 0.099833 0.11083];

% mean(pn1_mean_train)
% mean(pn1_sd_train)
% mean(pn2_mean_train)
% mean(pn2_sd_train)
% mean(pn3_mean_train)
% mean(pn3_sd_train)
% 
% mean(pn1_mean_test)
% mean(pn1_sd_test)
% mean(pn2_mean_test)
% mean(pn2_sd_test)
% mean(pn3_mean_test)
% mean(pn3_sd_test)

%% Calculate mean and std deviation parameter error over all cameras and all expriments with same pixel noise
pn1_mean_rot = [0.063843 0.37873 0.14306];
pn1_sd_rot = [0.061078 0.64933 0.22048];
pn1_mean_trans = [0.00016903 0.0003526 0.000361];
pn1_sd_trans = [0.00014048 0.00035404 0.000260];
pn2_mean_rot = [0.43035 0.96657 0.71123];
pn2_sd_rot = [0.73788 1.4753 0.56146];
pn2_mean_trans = [0.0009155 0.0016138 0.002395];
pn2_sd_trans = [0.00061912 0.0012028 0.0021085];
pn3_mean_rot = [1.0592 1.2462 2.4268];
pn3_sd_rot = [0.6761 1.6805 4.2643];
pn3_mean_trans = [0.0055534 0.0013724 0.0037737];
pn3_sd_trans = [0.0068998 0.00067915 0.0034965];

mean(pn1_mean_rot)
mean(pn1_sd_rot)
mean(pn1_mean_trans)
mean(pn1_sd_trans)
mean(pn2_mean_rot)
mean(pn2_sd_rot)
mean(pn2_mean_trans)
mean(pn2_sd_trans)
mean(pn3_mean_rot)
mean(pn3_sd_rot)
mean(pn3_mean_trans)
mean(pn3_sd_trans)