function [T_S_M, transform_chain] = movingToStaticChain(parameter_container, joint_angles, simulation_object)

num_dh_links = simulation_object.num_DH_links;
transform_chain = {}; %keep track of all transforms;

% Get the transformation parameters for the first 6dof transform between
% the moving camera and base frame.
if strcmp(simulation_object.link_struct(1).type,'mdh')
    link_opt = simulation_object.link_struct(1); % plus one because first link is 6dof transform
    link_theta = joint_angles(1);
    link_d = link_opt.default_values(2);
    link_a = link_opt.default_values(3);
    link_alpha = link_opt.default_values(4);
    link_beta = link_opt.default_values(5);
    link_y = link_opt.default_values(6);
    if(link_opt.index_map(1)>0) %theta
        idx = parameter_container.getKeyIndex('mdh_theta_1');
        link_theta = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(2)>0) %d
        idx = parameter_container.getKeyIndex('mdh_d_1');
        link_d = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(3)>0)%a
        idx = parameter_container.getKeyIndex('mdh_r_1');
        link_a = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(4)>0) %alpha
        idx = parameter_container.getKeyIndex('mdh_alpha_1');
        link_alpha = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(5)>0) % beta
        idx = parameter_container.getKeyIndex('mdh_beta_1');
        link_beta = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(6)>0) % y
        idx = parameter_container.getKeyIndex('mdh_y_1');
        link_y = parameter_container.parameter_list{idx}.parameter.value;
    end
    
    % Add offset to joint angles
    %theta_corrected = add_offsets_to_angles(link_theta, link_opt);
    theta_corrected = link_theta + link_opt.offset;
    
    %compute the transformation between this and the next joint.
    T_E_M = generateMDHMatrix(theta_corrected, link_d, link_a, link_alpha, link_beta, link_y);
    transform_chain{1} = T_E_M;
    
elseif strcmp(simulation_object.link_struct(1).type,'6dof')  
    idx = parameter_container.getKeyIndex('T_EM');
    T_E_M = parameter_container.parameter_list{idx}.parameter.matrix; % 6 dof from moving camera to end effector
    transform_chain{1} = T_E_M;
end

% Generate the intermediate transforms between the DH parameter links, for
% N total links.  Note that the BF frame is also the Q1 frame, or frame of
% the 1-st joint, and QN is the end effector or EE frame.
T_QN_E = eye(4);
for k=1:num_dh_links
    
    link_opt = simulation_object.link_struct(k+1); % plus one because first link is 6dof transform
    if simulation_object.use_modified_DH_flag
        k=k+1;
    end
    link_theta = joint_angles(k);
    link_d = link_opt.default_values(2);
    link_a = link_opt.default_values(3);
    link_alpha = link_opt.default_values(4);
    
    if(link_opt.index_map(1)>0) %theta
        idx = dh_key_idx('dh_theta_',k,parameter_container);
        link_theta = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(2)>0) %d
        idx = dh_key_idx('dh_d_',k,parameter_container);
        link_d = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(3)>0)%a
        idx = dh_key_idx('dh_r_',k,parameter_container);
        link_a = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(4)>0) %alpha
        idx = dh_key_idx('dh_alpha_',k,parameter_container);
        link_alpha = parameter_container.parameter_list{idx}.parameter.value;
    end
    
    % Add offset to joint angles
    %theta_corrected = add_offsets_to_angles(link_theta, link_opt);
    theta_corrected = link_theta + link_opt.offset;
    
    %compute the transformation between this and the next joint.
    T_Qk_Qkp1 = generateDHMatrix(theta_corrected, link_d, link_a, link_alpha);
    T_QN_E = T_Qk_Qkp1 * T_QN_E;
    transform_chain{end+1} = T_Qk_Qkp1;
end

static_cam_key = simulation_object.static_cam_key;

if isKey(parameter_container.parameter_key_map, static_cam_key)
    idx = parameter_container.getKeyIndex(static_cam_key);
    T_S_B = parameter_container.parameter_list{idx}.parameter.matrix; % 6 dof from base frame to static camera
    transform_chain{end+1} = T_S_B;
else
    static_cam_num = static_cam_key(end-1);
    link_opt = simulation_object.link_struct(1 + simulation_object.num_DH_links + str2double(static_cam_num));
    rx_val = link_opt.default_values(1);
    ry_val = link_opt.default_values(2);
    tx_val = link_opt.default_values(3);
    ty_val = link_opt.default_values(4);
    if(link_opt.index_map(1)>0)
        idx = dh_key_idx('4dof_rx_', str2double(static_cam_num), parameter_container);
        rx_val = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(2)>0)
        idx = dh_key_idx('4dof_ry_', str2double(static_cam_num), parameter_container);
        ry_val = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(3)>0)
        idx = dh_key_idx('4dof_tx_', str2double(static_cam_num), parameter_container);
        tx_val = parameter_container.parameter_list{idx}.parameter.value;
    end
    if(link_opt.index_map(4)>0)
        idx = dh_key_idx('4dof_ty_', str2double(static_cam_num), parameter_container);
        ty_val = parameter_container.parameter_list{idx}.parameter.value;
    end
    T_S_B = generate4dofMatrix(rx_val,ry_val,tx_val,ty_val);
    transform_chain{end+1} = T_S_B;
end

% Add the 6 DOF transforms. Transformation from static camera to moving
% camera
T_S_M = T_S_B * T_QN_E * T_E_M;
end
