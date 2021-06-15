function BWR_callback(source, event, handles)

    % This call back is used to rotate the base of the drone so that
    % cameras pointing in the oppositie direction will be able to see the
    % target
    global input_angles dsc_obj sliders orig_T_WB
    figure(sliders)
    y_pos = source.Position(2)-30;
    actual_angle_deg = source.Value-180;
    uicontrol('Style','text','Position',[320,y_pos,80,23],...
            'String',strcat("= ",num2str(actual_angle_deg)));
    dh_rad = deg2rad(input_angles);
    dsc_obj.transforms.world_T_base = orig_T_WB*Transformation([0,0,deg2rad(actual_angle_deg),0,0,0]).matrix;
    showObjAndPix(dsc_obj, dh_rad);
end