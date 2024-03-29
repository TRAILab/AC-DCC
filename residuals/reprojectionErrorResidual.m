function [r, J_Total] = reprojectionErrorResidual(camera, T_c_a, a_p, c_pix)

%% Description
% This file determines the jacobian and residual for the re-projection
% error

[c_p_predicted, J__T_c_b,~] = T_c_a.transformAndJacobian(a_p);

% perform projection
[pixel_projection, J_proj] = camera.projectAndJacobian(c_p_predicted);

% compute residual if valid projection
if (~isempty(pixel_projection))
    r = c_pix - pixel_projection;
    r = reshape(r, [2,1]);
    % Total residual using chain rule.
    J_Total = -1*J_proj*J__T_c_b;
else
    r = [];
    J_Total=[];
end

