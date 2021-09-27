function error = getL2Error(A, B)

%% Description
% This returns the L2 error statistics between two vectors

error_matrix = A - B;
error_vector = sqrt(sum(error_matrix.^2,2));
error.mean = mean(error_vector);
error.max = max(error_vector);
error.min = min(error_vector);