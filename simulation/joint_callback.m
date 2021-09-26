function joint_callback(source, event, handles)

%% Descriptions
% This is the callback for the joint angles

    % Callback for sliders
    global input_angles dsc_obj sliders
    figure(sliders)
    joint_num = str2double(source.String(end));
    y_pos = source.Position(2)-30;
    if dsc_obj.use_modified_DH_flag
        actual_angle_deg = source.Value + rad2deg(dsc_obj.link_struct(joint_num + 1).bounds(1));
    else
        actual_angle_deg = source.Value + rad2deg(dsc_obj.link_struct(joint_num + 2).bounds(1));
    end
    uicontrol('Style', 'text', 'Position', [320,y_pos,80,23], 'String', strcat("= ",num2str(actual_angle_deg)));
    
    % subtract lower angle bound
    input_angles(joint_num+1) = actual_angle_deg;
    dh_rad = deg2rad(input_angles);
    showObjAndPix(dsc_obj, dh_rad);
end