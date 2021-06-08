function [T_SM, transform_chain] = movingToStaticChain(parameter_container, joint_angles, dcc_obj)

num_dh_links = dcc_obj.num_DH_links;
transform_chain = {}; %keep track of all transforms;

% Get the transformation parameters for the first 6dof transform between
% the moving camera and base frame.
if strcmp(dcc_obj.link_struct(1).type,'mdh')
    link_opt = dcc_obj.link_struct(1); % plus one because first link is 6dof transform
    dv = link_opt.default_values;
    link_theta = joint_angles(1);
    [link_d, link_a, link_alpha, link_beta, link_y] = deal(dv(2), dv(3), dv(4), dv(5), dv(6));
    if(link_opt.index_map(2)>0) %d
        link_d = parameter_container.getKeyValue('mdh_d_1');
    end
    if(link_opt.index_map(3)>0)%a
        link_a = parameter_container.getKeyValue('mdh_r_1');
    end
    if(link_opt.index_map(4)>0) %alpha
        link_alpha = parameter_container.getKeyValue('mdh_alpha_1');
    end
    if(link_opt.index_map(5)>0) % beta
        link_beta = parameter_container.getKeyValue('mdh_beta_1');
    end
    if(link_opt.index_map(6)>0) % y
        link_y = parameter_container.getKeyValue('mdh_y_1');
    end
    
    % Add offset to joint angles
    theta_corrected = link_theta + link_opt.offset;
    
    %compute the transformation between this and the next joint.
    T_E_M = generateModifiedDHMatrix(theta_corrected, link_d, link_a, link_alpha, link_beta, link_y);
    transform_chain{1} = T_E_M;
    
elseif strcmp(dcc_obj.link_struct(1).type,'6dof')  
    T_E_M = parameter_container.getKeyValue('T_EM'); % 6 dof from moving camera to end effector
    transform_chain{1} = T_E_M;
end

% Generate the intermediate transforms between the DH parameter links, for
% N total links.  Note that the BF frame is also the Q1 frame, or frame of
% the 1-st joint, and QN is the end effector or EE frame.
T_QN_E = eye(4);
for k=1:num_dh_links
    
    link_opt = dcc_obj.link_struct(k+1); % plus one because first link is 6dof transform
    if dcc_obj.use_modified_DH_flag
        k=k+1;
    end
    link_theta = joint_angles(k);
    dv = link_opt.default_values;
    [link_d, link_a, link_alpha] = deal(dv(2), dv(3), dv(4));
    
    if(link_opt.index_map(2)>0) %d
        link_d = parameter_container.getKeyValue(strcat('dh_d_',num2str(k)));
    end
    if(link_opt.index_map(3)>0)%a
        link_a = parameter_container.getKeyValue(strcat('dh_r_',num2str(k)));
    end
    if(link_opt.index_map(4)>0) %alpha
        link_alpha = parameter_container.getKeyValue(strcat('dh_alpha_',num2str(k)));
    end
    
    % Add offset to joint angles
    theta_corrected = link_theta + link_opt.offset;
    
    %compute the transformation between this and the next joint.
    T_Qk_Qkp1 = generateDHMatrix(theta_corrected, link_d, link_a, link_alpha);
    T_QN_E = T_Qk_Qkp1 * T_QN_E;
    transform_chain{end+1} = T_Qk_Qkp1;
end

static_cam_key = dcc_obj.static_cam_key;

if isKey(parameter_container.parameter_key_map, static_cam_key)
    T_SB = parameter_container.getKeyValue(static_cam_key); 
    transform_chain{end+1} = T_SB;
else
    static_cam_num = static_cam_key(end-1);
    link_opt = dcc_obj.link_struct(1 + dcc_obj.num_DH_links + str2double(static_cam_num));
    dv = link_opt.default_values;
    [rx_val, ry_val, tx_val, ty_val] = deal(dv(1), dv(2), dv(3), dv(4));
    if(link_opt.index_map(1)>0)
        rx_val = parameter_container.getKeyValue(strcat('4dof_rx_', static_cam_num));
    end
    if(link_opt.index_map(2)>0)
        ry_val = parameter_container.getKeyValue(strcat('4dof_ry_', static_cam_num));
    end
    if(link_opt.index_map(3)>0)
        tx_val = parameter_container.getKeyValue(strcat('4dof_tx_', static_cam_num));
    end
    if(link_opt.index_map(4)>0)
        ty_val = parameter_container.getKeyValue(strcat('4dof_ty_', static_cam_num));
    end
    T_SB = generate4dofMatrix(rx_val,ry_val,tx_val,ty_val);
    transform_chain{end+1} = T_SB;
end

% Add the 6 DOF transforms. Transformation from static camera to moving camera
T_SM = T_SB * T_QN_E * T_E_M;
end
