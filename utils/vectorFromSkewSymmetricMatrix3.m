function [ v ] = vectorFromSkewSymmetricMatrix3( ssm )

v(1) = ssm(3,2);
v(2) = ssm(1,3);
v(3) = ssm(2,1);

v = v';
end

