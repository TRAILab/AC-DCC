function [T_SM, transform_chain] = movingToStaticChain(parameter_container, joint_angles, dcc_obj)
%% This function returns the transformation from dynamic camera to static
% camera based on the static camera key. 
% It also returns individual transformation matrics for each link starting 
% from the dynamic camera.

num_dh_links = dcc_obj.num_DH_links;
transform_chain = {}; % keep track of all transforms;

% Gets the transformation parameters from dynamic camera to previous joint
% based on the parameterization used
if strcmp(dcc_obj.link_struct(1).type,'mdh')
    link_opt = dcc_obj.link_struct(1); % plus one because first link is 6dof transform
    dv = link_opt.default_values;
    link_theta = joint_angles(1);
    [link_d, link_a, link_alpha, link_beta, link_y] = deal(dv(2), dv(3), dv(4), dv(5), dv(6));
    if(link_opt.index_map(2)>0) % d
        link_d = parameter_container.getKeyValue('mdh_d_1');
    end
    if(link_opt.index_map(3)>0) % a (also known as r parameter)
        link_a = parameter_container.getKeyValue('mdh_r_1');
    end
    if(link_opt.index_map(4)>0) % alpha
        link_alpha = parameter_container.getKeyValue('mdh_alpha_1');
    end
    if(link_opt.index_map(5)>0) % beta
        link_beta = parameter_container.getKeyValue('mdh_beta_1');
    end
    if(link_opt.index_map(6)>0) % y
        link_y = parameter_container.getKeyValue('mdh_y_1');
    end
    
    % Adds offset to joint angles
    theta_corrected = link_theta + link_opt.offset;
    
    % Computes the transformation from dynamic camera to previous joint
    T_J3M = generateModifiedDHMatrix(theta_corrected, link_d, link_a, link_alpha, link_beta, link_y);
    transform_chain{1} = T_J3M;
    
elseif strcmp(dcc_obj.link_struct(1).type,'6dof')  
    T_J3M = parameter_container.getKeyValue('T_EM');
    transform_chain{1} = T_J3M;
end

% Generate the intermediate transforms between the DH parameter links, for
% N total links. 
T_BJ3 = eye(4);
for k=1:num_dh_links
    
    link_opt = dcc_obj.link_struct(k+1); % plus one because first link is 6dof transform
    if dcc_obj.use_modified_DH_flag
        k=k+1;
    end
    link_theta = joint_angles(k);
    dv = link_opt.default_values;
    [link_d, link_a, link_alpha] = deal(dv(2), dv(3), dv(4));
    
    if(link_opt.index_map(2)>0) % d
        link_d = parameter_container.getKeyValue(strcat('dh_d_',num2str(k)));
    end
    if(link_opt.index_map(3)>0) % a or r parameter
        link_a = parameter_container.getKeyValue(strcat('dh_r_',num2str(k)));
    end
    if(link_opt.index_map(4)>0) % alpha
        link_alpha = parameter_container.getKeyValue(strcat('dh_alpha_',num2str(k)));
    end
    
    % Adds offset to joint angles
    theta_corrected = link_theta + link_opt.offset;
    
    % Computes the transformation between this and the next joint.
    T_Qk_Qkp1 = generateDHMatrix(theta_corrected, link_d, link_a, link_alpha);
    T_BJ3 = T_Qk_Qkp1 * T_BJ3;
    transform_chain{end+1} = T_Qk_Qkp1;
end

% Gets the information on which static camera to process
static_cam_key = dcc_obj.static_cam_key;
static_cam_num_str = static_cam_key(end-1);
link_opt = dcc_obj.link_struct(1 + dcc_obj.num_DH_links + str2double(static_cam_num_str));
dv = link_opt.default_values;

% Since this is the first static camera, we use the 4-DOF
if strcmp(static_cam_num_str,'1')
    [rx_val, ry_val, tx_val, ty_val] = deal(dv(1), dv(2), dv(3), dv(4));
    if(link_opt.index_map(1)>0)
        rx_val = parameter_container.getKeyValue(strcat('4dof_rx_', static_cam_num_str));
    end
    if(link_opt.index_map(2)>0)
        ry_val = parameter_container.getKeyValue(strcat('4dof_ry_', static_cam_num_str));
    end
    if(link_opt.index_map(3)>0)
        tx_val = parameter_container.getKeyValue(strcat('4dof_tx_', static_cam_num_str));
    end
    if(link_opt.index_map(4)>0)
        ty_val = parameter_container.getKeyValue(strcat('4dof_ty_', static_cam_num_str));
    end
    T_SB = generate4dofMatrix(rx_val,ry_val,tx_val,ty_val);
    transform_chain{end+1} = T_SB;
else
    if isKey(parameter_container.parameter_key_map, static_cam_key)
        T_SB = parameter_container.getKeyValue(static_cam_key); 
        transform_chain{end+1} = T_SB;
    else
        T_SB = Transformation(dv).matrix; 
        transform_chain{end+1} = T_SB;
    end
    
end

% Get the entire transformation from dynamic camera to static camera
T_SM = T_SB * T_BJ3 * T_J3M;
