function setupUI(sim_obj)

%% Description
% Setup UI to control the simulation object

global sliders

sliders = figure('Name','UI');
disp("===========================================");
disp('Please note the angles J0, J1, J2 .. start from the last joint (the one closest to the end effector) and proceed to the base joint');
disp(strcat("Encoder Noise (Deg): Mean ", num2str(rad2deg(sim_obj.encoder_noise.mean)), " Std Dev ", num2str(rad2deg(sim_obj.encoder_noise.std_dev))));
disp("===========================================");
combined_DH_links = sim_obj.num_DH_links + sim_obj.use_modified_DH_flag;
myhandles = guihandles(sliders);
myhandles.simulation_object = sim_obj;
myhandles.angles = zeros(1,combined_DH_links);
guidata(sliders,myhandles)
names = ["J0","J1","J2","J3","J4","J5","J6"];
ui_control_vec = [];

a = 1:combined_DH_links;
strt1 = 350;
strt2 = strt1-30;

% This are the sliders for the individual joint axis
for i=1:length(a)
    
    if sim_obj.use_modified_DH_flag
        bounds = rad2deg(sim_obj.link_struct(i).bounds);
        ui_control_vec(i) = uicontrol('Parent',sliders,'String',names(i),'Style','slider','Position',...
                                      [80,strt1-70*(a(i)-1),419,23],'value',(-bounds(1)+bounds(2))/2,...
                                      'min',0, 'max',-bounds(1)+bounds(2),'Callback',@joint_callback);
        b = uicontrol('Parent',sliders,'Style','text','Position',[250,strt2-70*(a(i)-1),100,23],...
                    'String',names(i));
        c = uicontrol('Parent',sliders,'Style','text','Position',[20,strt2-70*(a(i)-1),50,50],...
                    'String',num2str(bounds(1)));
        d = uicontrol('Parent',sliders,'Style','text','Position',[500,strt2-70*(a(i)-1),50,50],...
                    'String',num2str(bounds(2)));
    else
        bounds = rad2deg(sim_obj.link_struct(i+1).bounds);
        ui_control_vec(i) = uicontrol('Parent',sliders,'String',names(i),'Style','slider','Position',...
                                      [80,strt1-70*(a(i)-1),419,23],'value',(-bounds(1)+bounds(2))/2,...
                                      'min',0, 'max',-bounds(1)+bounds(2),'Callback',@joint_callback);
        b = uicontrol('Parent',sliders,'Style','text','Position',[250,strt2-70*(a(i)-1),100,23],...
                    'String',names(i));
        c = uicontrol('Parent',sliders,'Style','text','Position',[20,strt2-70*(a(i)-1),50,50],...
                    'String',num2str(bounds(1)));
        d = uicontrol('Parent',sliders,'Style','text','Position',[500,strt2-70*(a(i)-1),50,50],...
                    'String',num2str(bounds(2)));
    end
end

% This is the slider for moving the base in the world frame. This is mainly
% used when we have non-overlapping static cameras, to move the base so
% that other static cameras can view the target points. Similar to rotating
% the drone about the yaw axis.

bounds = [-180 180];
i=i+1;
a = [a a(end)+1];
ui_control_vec(i) = uicontrol('Parent',sliders,'String','Base World Rotation','Style','slider','Position',...
                              [80,strt1-70*(a(i)-1),419,23],'value',(-bounds(1)+bounds(2))/2,...
                              'min',0, 'max',-bounds(1)+bounds(2),'Callback',@BWR_callback);
b = uicontrol('Parent',sliders,'Style','text','Position',[250,strt2-70*(a(i)-1),150,23],...
            'String','Base World Rotation');
c = uicontrol('Parent',sliders,'Style','text','Position',[20,strt2-70*(a(i)-1),50,50],...
            'String',num2str(bounds(1)));
d = uicontrol('Parent',sliders,'Style','text','Position',[500,strt2-70*(a(i)-1),50,50],...
            'String',num2str(bounds(2)));