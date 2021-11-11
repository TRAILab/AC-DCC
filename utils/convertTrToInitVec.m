function init_vec = convertTrToInitVec(T)

%% Description
% This function converts a transformation matrix object into a vector for
% initialization.

vec2 = tran2vec2(T);
vec2(1:3) = rad2deg(vec2(1:3));
init_vec = vec2;