function total_angles = getTotalAngles(link_struct)
%% Returns the total number of angles (joints) by analyzing link struct

total_angles = 0;
for i=1:length(link_struct)
    if ~isempty(link_struct(i).bounds)
        total_angles = total_angles+1;
    end
end