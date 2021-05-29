function S = skewSymmetricMatrix3(input_vector)

S =[0 -input_vector(3) input_vector(2) ; input_vector(3) 0 -input_vector(1) ; -input_vector(2) input_vector(1) 0 ];

end
