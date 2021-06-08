function setup_ui(simulation_object)

% UI to display sliders

global sliders

sliders = figure(5);
disp('Please note the angles J0, J1, J2 .. start from the last joint (the one closest to the end effector) and proceed to the base joint');
combined_DH_links = simulation_object.num_DH_links + simulation_object.use_modified_DH_flag;
myhandles = guihandles(sliders);
myhandles.simulation_object = simulation_object;
myhandles.angles = zeros(1,combined_DH_links);
guidata(sliders,myhandles)
names = ["J0","J1","J2","J3","J4","J5","J6"];
ui_control_vec = [];

a = 1:combined_DH_links;
strt1 = 350;
strt2 = strt1-30;

for i=1:length(a)
    
    if simulation_object.use_modified_DH_flag
        bounds = rad2deg(simulation_object.link_struct(i).bounds);
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
        bounds = rad2deg(simulation_object.link_struct(i+1).bounds);
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