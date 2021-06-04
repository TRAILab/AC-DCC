function [ chain_helper ] = generateChainHelper(transform_chain)

% For regular DH with last 6 dof joint
% transform_chain: T_ED, T_J3E, T_J2J3, T_BJ2, T_SB
% post --- analyzed link --- pre 
% T_SE --- T_ED --- I
% T_SJ3 --- T_J3E --- T_ED
% T_SJ2 --- T_J2J3 --- T_J3D  
% T_SB --- T_BJ2 --- T_J2D  
% I --- T_SB --- T_BD

% For DHM on the last joint
% transform_chain: T_ED, T_J2E, T_BJ2, T_SB
% post --- analyzed --- pre
% T_SE --- T_ED --- I
% T_SJ2 --- T_J2E --- T_ED  
% T_SB --- T_BJ2 --- T_J2D 
% I --- T_SB --- T_BD

chain_helper = {};
chain_length = length(transform_chain);

for i=1:chain_length
    
    chain.pre = eye(4);
    chain.post = eye(4);
    
    num_to_current = i-1;
    num_to_post = chain_length-i;
    
    for j=1:num_to_current
        chain.pre = transform_chain{j}*chain.pre;
    end
    
    for j=1:num_to_post
        chain.post = transform_chain{i+j}*chain.post;
    end
    chain_helper{i} = chain;
end

end

