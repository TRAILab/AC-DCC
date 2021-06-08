function joint_callback(source,event,handles)

        % Callback for sliders
        global input_angles simz_obj sliders angle_conversion_name comb_figs
        figure(sliders)
        myhandles = guidata(gcbo);
        joint_num = str2num(source.String(end));
        y_pos = source.Position(2)-30;
        if sim_obj.use_modified_DH_flag
            actual_angle_deg = source.Value+rad2deg(sim_obj.link_struct(joint_num+1).bounds(1));
        else
            actual_angle_deg = source.Value+rad2deg(sim_obj.link_struct(joint_num+2).bounds(1));
        end
        uicontrol('Style','text','Position',[320,y_pos,80,23],...
                'String',strcat("= ",num2str(actual_angle_deg)));
        % subtract lower angle bound
        input_angles(joint_num+1) = actual_angle_deg;
        %dh_deg = angle_conversion(angle_conversion_name,input_angles);
        dh_rad = deg2rad(input_angles);
        show_obj_and_pix(sim_obj, dh_rad);
        %display_only_arm(simulation_object,dh_rad);
end
