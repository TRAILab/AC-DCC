function [p] = tran2vec2(T)

tranValidate(T);

C = T(1:3,1:3);
r = T(1:3,4);

phi = rot2vec(C);

%invJ = vec2jacInv(phi);
%invJ = vec2jacInvSeries(phi,10);

%rho = invJ * r;
%p = [rho;phi];
p = [phi; r];
end