function [ chain_helper ] = generateChainHelper(transform_chain )

% We will create a datastructure that will help manage the transform chains
% required to generate the jacobians. Each element will have a current field,
% which is the transform that maps a point into that current frame, as well
% as a post field, which is the remainder of the chain NOT INCLUDING the
% current transform.

% For regular DH with last 6 dof joint
% transform_chain: T_ED, T_J3E, T_J2J3, T_BJ2, T_SB
% analyzed link --- pre --- post % Easy reading if we make it pre --- analyzed --- post
% T_ED --- I --- T_SE
% T_J3E --- T_ED --- T_SJ3
% T_J2J3 --- T_J3D --- T_SJ2
% T_BJ2 --- T_J2D --- T_SB
% T_SB --- T_BD --- I

% For DHM on the last joint
% transform_chain: T_ED, T_J2E, T_BJ2, T_SB
% pre --- analyzed --- post
% I --- T_ED --- T_SE
% T_ED --- T_J2E --- T_SJ2
% T_J2D --- T_BJ2 --- T_SB
% T_BD --- T_SB --- I

chain_helper = {};
chain_length = length(transform_chain);

for i=1:chain_length
    
    chain.pre = eye(4);
    chain.post = eye(4);
    
    num_to_current = i-1;
    num_to_post = chain_length-i;
    
    for j=1:num_to_current
        %j
        chain.pre = transform_chain{j}*chain.pre;
    end
    
    for j=1:num_to_post
        %i+j
        chain.post = transform_chain{i+j}*chain.post;
    end
    chain_helper{i} = chain;
end

end

