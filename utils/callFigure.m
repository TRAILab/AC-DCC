function callFigure(figure_name)

% This function is just a helper to call the figure if it exists

if isempty(findobj('type','figure','name',figure_name))
    figure('Name',figure_name);
else
    figure(findobj('type','figure','name',figure_name).Number);
end