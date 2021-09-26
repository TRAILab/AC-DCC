function opt_problem = setupOptimizationProblem(dcc_obj, measurement_set)

%% Description:
% This function sets up the optimization problem for the calibration

% set up the parameter container
parameter_container = ParameterContainer;
parameter_container.parameter_key_map = containers.Map; % we do this because even though we are creating a new ParameterContainer, 
                                                        % matlab still holds old containers and therefore we need to reset

if strcmp(dcc_obj.link_struct(1).type,'mdh')
    % Add the moving camera to end effector modified 6dof dh
    idx_map = dcc_obj.link_struct(1).index_map;
    dvs = dcc_obj.link_struct(1).default_values;
    
    assert(length(idx_map)==6, 'idx_map wrong length!');
    
    % We start with the d parameter and handle the joint angles later
    if(idx_map(2)>0) % d parameter
        d_opt_param = OptimizationParameter(MDHParamD(dvs(2)), 1, 'mdh_d_1');
        parameter_container.addParameter(d_opt_param);
    end
    if(idx_map(3)>0) % r parameter
        r_opt_param = OptimizationParameter(MDHParamR(dvs(3)), 1, 'mdh_r_1');
        parameter_container.addParameter(r_opt_param);
    end
    if(idx_map(4)>0) % alpha parameter
        alpha_opt_param = OptimizationParameter(MDHParamAlpha(dvs(4)), 1, 'mdh_alpha_1');
        parameter_container.addParameter(alpha_opt_param);
    end
    if(idx_map(5)>0) % beta parameter
        beta_opt_param = OptimizationParameter(MDHParamBeta(dvs(5)), 1, 'mdh_beta_1');
        parameter_container.addParameter(beta_opt_param);
    end
    if(idx_map(6)>0) % y parameter
        y_opt_param = OptimizationParameter(MDHParamY(dvs(6)), 1, 'mdh_y_1');
        parameter_container.addParameter(y_opt_param);
    end
    
elseif strcmp(dcc_obj.link_struct(1).type,'6dof')                                               
    % Add the moving camera to end effector transformation to the parameter container
    dvs = dcc_obj.link_struct(1).default_values;
    T_M =  Transformation([dvs(1) dvs(2) dvs(3) dvs(4) dvs(5) dvs(6)]);
    O_M = OptimizationParameter(T_M, 6, 'T_EM');
    parameter_container.addParameter(O_M);
end

%dh_variables = {};
num_dh_links = dcc_obj.num_DH_links;
for i=1:num_dh_links
    
    % These are assumed to be of size 4, since they are DH links
    % we have them in the parameter file order of static->moving, but we
    % want to add them in the order of moving->static
    idx_map = dcc_obj.link_struct(i+1).index_map;
    dvs = dcc_obj.link_struct(i+1).default_values;
    
    assert(length(idx_map)==4, 'idx_map wrong length!');
    
    if dcc_obj.use_modified_DH_flag
        i = i+1;
    end
    
    if(idx_map(2)>0) % d parameter
        key = strcat('dh_d_', int2str(i));
        d_opt_param = OptimizationParameter(DHParamD(dvs(2)), 1, key);
        parameter_container.addParameter(d_opt_param);
    end
    if(idx_map(3)>0) % r parameter
        key = strcat('dh_r_', int2str(i));
        r_opt_param = OptimizationParameter(DHParamR(dvs(3)), 1, key);
        parameter_container.addParameter(r_opt_param);
    end
    if(idx_map(4)>0) % alpha parameter
        key = strcat('dh_alpha_', int2str(i));
        alpha_opt_param = OptimizationParameter(DHParamAlpha(dvs(4)), 1, key);
        parameter_container.addParameter(alpha_opt_param);
    end
end

% Add the last 6dof parameter block
static_count = 1;
for m=1+num_dh_links+1:length(dcc_obj.link_struct)
    link_struct = dcc_obj.link_struct(m);
    dvs = link_struct.default_values; % first one is the static camera
    idx_map = link_struct.index_map;
    if strcmp(link_struct.type,'4dof')
        if(idx_map(1)>0) % alpha parameter
            key = strcat('4dof_rx_', num2str(static_count));
            rx_opt_param = OptimizationParameter(Rx4DofParam(dvs(1)), 1, key);
            parameter_container.addParameter(rx_opt_param);
        end
        if(idx_map(2)>0) % alpha parameter
            key = strcat('4dof_ry_', num2str(static_count));
            ry_opt_param = OptimizationParameter(Ry4DofParam(dvs(2)), 1, key);
            parameter_container.addParameter(ry_opt_param);
        end
        if(idx_map(3)>0) % alpha parameter
            key = strcat('4dof_tx_', num2str(static_count));
            tx_opt_param = OptimizationParameter(Tx4DofParam(dvs(3)), 1, key);
            parameter_container.addParameter(tx_opt_param);
        end
        if(idx_map(4)>0) % alpha parameter
            key = strcat('4dof_ty_', num2str(static_count));
            ty_opt_param = OptimizationParameter(Ty4DofParam(dvs(4)), 1, key);
            parameter_container.addParameter(ty_opt_param);
        end
    end
    if strcmp(link_struct.type,'6dof')
        T_S = Transformation([dvs(1) dvs(2) dvs(3) dvs(4) dvs(5) dvs(6)]);
        if any(idx_map>-1)
            O_S = OptimizationParameter(T_S, 6, strcat('T_S',num2str(static_count),'B'));
            parameter_container.addParameter(O_S);
        end
    end
    static_count = static_count+1;
end

% Add each angle as the optimization parameter.
if dcc_obj.optimize_theta_flag && ~isempty(measurement_set)
    theta_flag_vec = dcc_obj.optimize_theta_flag_vec;
    for m=1:length(measurement_set)
        theta_vec = measurement_set{m}.theta_vec;
        for t = 1 : num_dh_links + dcc_obj.use_modified_DH_flag
            theta_val = theta_vec(t);
            if theta_flag_vec(t) == 1
                if t==1 && dcc_obj.use_modified_DH_flag
                    key = strcat('mdh_theta_', int2str(m), '_', int2str(t));
                    theta_opt_param = OptimizationParameter(MDHParamTheta(theta_val), 1, key);
                else
                    key = strcat('dh_theta_', int2str(m), '_', int2str(t));
                    theta_opt_param = OptimizationParameter(DHParamTheta(theta_val), 1, key);
                end
                parameter_container.addParameter(theta_opt_param);
            end
        end
    end
end

% set up the optimization problem
opt_problem = Problem(parameter_container);