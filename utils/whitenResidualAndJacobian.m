function [r_whiten, J_whiten] = whitenResidualAndJacobian(residual, Jacobian, info_matrix)
% prewhitens the residual, so that we can account for the noise
% characteristics in the optimization.

% Take the cholesky factorization of this iformation matrix.  We perform
% the following math:
%  L*L' = chol(info_matrix)
% H = J'*info_matrix*J
% H = J'*(L*L')*J
% H = (J'*L)*(L'*J)
% H = G'*G, where G =(L'*J)
% thus, G is our prewhitened Jacobian.  A similar process is performed for
% the residual vector.
L = chol(info_matrix,'lower'); 
J_whiten = L'*Jacobian;
r_whiten = L'*residual;
