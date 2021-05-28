function opt_problem = setupOptimizationProblem(simulation_object, measurement_set)

%% This file sets up the optimization problem

% set up the parameter container
parameter_container = ParameterContainer;
parameter_container.parameter_key_map = containers.Map; % we do this because even though we are creating a new ParameterContainer, 
                                                        % matlab still holds old containers and therefore we need to reset

if strcmp(simulation_object.link_struct(1).type,'mdh')
    % Add the moving camera to end effector modified 6dof dh
    idx_map = simulation_object.link_struct(1).index_map;
    default_values = simulation_object.link_struct(1).default_values;
    if length(idx_map)~=6
        error('idx_map wrong length!');
    end
    
    if(idx_map(1)>0) % theta parameter
        key = strcat('mdh_theta_1');
        theta_opt_param = OptimizationParameter(MDHParamTheta(default_values(1)),1,key);
        parameter_container.addParameter(theta_opt_param);
    end
    if(idx_map(2)>0) % d parameter
        key = strcat('mdh_d_1');
        d_opt_param = OptimizationParameter(MDHParamD(default_values(2)),1,key);
        parameter_container.addParameter(d_opt_param);
    end
    if(idx_map(3)>0) % r parameter
        key = strcat('mdh_r_1');
        r_opt_param = OptimizationParameter(MDHParamR(default_values(3)),1,key);
        parameter_container.addParameter(r_opt_param);
    end
    if(idx_map(4)>0) % alpha parameter
        key = strcat('mdh_alpha_1');
        alpha_opt_param = OptimizationParameter(MDHParamAlpha(default_values(4)),1,key);
        parameter_container.addParameter(alpha_opt_param);
    end
    if(idx_map(5)>0) % beta parameter
        key = strcat('mdh_beta_1');
        beta_opt_param = OptimizationParameter(MDHParamBeta(default_values(5)),1,key);
        parameter_container.addParameter(beta_opt_param);
    end
    if(idx_map(6)>0) % y parameter
        key = strcat('mdh_y_1');
        y_opt_param = OptimizationParameter(MDHParamY(default_values(6)),1,key);
        parameter_container.addParameter(y_opt_param);
    end
    
elseif strcmp(simulation_object.link_struct(1).type,'6dof')                                               
    % Add the moving camera to end effector transformation to the parameter container
    default_values = simulation_object.link_struct(1).default_values;
    T_M =  Transformation([default_values(1) default_values(2) default_values(3) default_values(4) default_values(5) default_values(6)]);
    O_M = OptimizationParameter(T_M, 6, 'T_EM');
    parameter_container.addParameter(O_M);
end

%dh_variables = {};
num_dh_links = simulation_object.num_DH_links;
for i=1:num_dh_links
    
    % These are assumed to be of size 4, since they are DH links
    % we have them in the parameter file order of static->moving, but we
    % want to add them in the order of moving->static
    idx_map = simulation_object.link_struct(i+1).index_map;
    default_values = simulation_object.link_struct(i+1).default_values;
    
    if length(idx_map)~=4
        error('idx_map wrong length!');
    end
    
    if simulation_object.use_modified_DH_flag
        i = i+1;
    end
    
    if(idx_map(1)>0) % theta parameter
        key = strcat('dh_theta_',int2str(i));
        theta_opt_param = OptimizationParameter(DHParamTheta(default_values(1)), 1, key);
        parameter_container.addParameter(theta_opt_param);
    end
    if(idx_map(2)>0) % d parameter
        key = strcat('dh_d_',int2str(i));
        d_opt_param = OptimizationParameter(DHParamD(default_values(2)), 1, key);
        parameter_container.addParameter(d_opt_param);
    end
    if(idx_map(3)>0) % r parameter
        key = strcat('dh_r_',int2str(i));
        r_opt_param = OptimizationParameter(DHParamR(default_values(3)), 1, key);
        parameter_container.addParameter(r_opt_param);
    end
    if(idx_map(4)>0) % alpha parameter
        key = strcat('dh_alpha_',int2str(i));
        alpha_opt_param = OptimizationParameter(DHParamAlpha(default_values(4)), 1, key);
        parameter_container.addParameter(alpha_opt_param);
    end
end

% Add the last 6dof parameter block
static_count = 1;
for m=1+num_dh_links+1:length(simulation_object.link_struct)
    default_values = simulation_object.link_struct(m).default_values; % first one is the static camera
    idx_map = simulation_object.link_struct(m).index_map;
    if strcmp(simulation_object.link_struct(m).type,'4dof')
        if(idx_map(1)>0) % alpha parameter
            key = strcat('4dof_rx_',num2str(static_count));
            rx_opt_param = OptimizationParameter(Rx4DofParam(default_values(1)), 1, key);
            parameter_container.addParameter(rx_opt_param);
        end
        if(idx_map(2)>0) % alpha parameter
            key = strcat('4dof_ry_',num2str(static_count));
            ry_opt_param = OptimizationParameter(Ry4DofParam(default_values(2)), 1, key);
            parameter_container.addParameter(ry_opt_param);
        end
        if(idx_map(3)>0) % alpha parameter
            key = strcat('4dof_tx_',num2str(static_count));
            tx_opt_param = OptimizationParameter(Tx4DofParam(default_values(3)), 1, key);
            parameter_container.addParameter(tx_opt_param);
        end
        if(idx_map(4)>0) % alpha parameter
            key = strcat('4dof_ty_',num2str(static_count));
            ty_opt_param = OptimizationParameter(Ty4DofParam(default_values(4)), 1, key);
            parameter_container.addParameter(ty_opt_param);
        end
    end
    if strcmp(simulation_object.link_struct(m).type,'6dof')
        T_S = Transformation([default_values(1) default_values(2) default_values(3) default_values(4) default_values(5) default_values(6)]);
        O_S = OptimizationParameter(T_S, 6, strcat('T_S',num2str(static_count),'B'));
        parameter_container.addParameter(O_S);
    end
    static_count = static_count+1;
end

% Add each angle as the optimization parameter.
if simulation_object.optimize_theta_flag && ~isempty(measurement_set)
    theta_flag_vec = simulation_object.optimize_theta_flag_vec;
    for m=1:length(measurement_set)
        theta_vec = measurement_set{m}.theta_vec;
        
        if simulation_object.use_modified_DH_flag
            for t=1:num_dh_links + simulation_object.use_modified_DH_flag
                if theta_flag_vec(t) == 1 
                    theta_val = theta_vec(t);
                    if t==1 && simulation_object.use_modified_DH_flag % This is the mdh value
                        key = strcat('mdh_theta_', int2str(m), '_', int2str(t));
                        theta_opt_param = OptimizationParameter(MDHParamTheta(theta_val), 1, key);
                    else
                        key = strcat('dh_theta_', int2str(m), '_', int2str(t));
                        theta_opt_param = OptimizationParameter(DHParamTheta(theta_val), 1, key);
                    end
                    parameter_container.addParameter(theta_opt_param);
                end
            end
        else
            for t=1:num_dh_links
                if theta_flag_vec(t) == 1
                    theta_val = theta_vec(t);
                    key = strcat('dh_theta_',int2str(m),'_',int2str(t));
                    theta_opt_param = OptimizationParameter(DHParamTheta(theta_val), 1, key);
                    parameter_container.addParameter(theta_opt_param);
                end
            end
        end
        
    end
end

% set up the optimization problem
opt_problem = Problem(parameter_container);
