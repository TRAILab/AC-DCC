
function S = skewSymmetricMatrix6(input_vector)
w =[0 -input_vector(3) input_vector(2) ; input_vector(3) 0 -input_vector(1) ; -input_vector(2) input_vector(1) 0 ];
u = [input_vector(4);input_vector(5);input_vector(6)];
S = zeros(4,4);
S(1:3,1:3) = w;
S(1:3,4) = u;
end
