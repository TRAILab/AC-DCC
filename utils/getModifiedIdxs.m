function shifted_idxs = getModifiedIdxs(bad_meas_idxs, max_idxs)

%% Description
% This returns the shifted index based on bad measurement indices

shifted_idxs = zeros(length(max_idxs),1);

for i=1:length(max_idxs)
    curr_max_idx = max_idxs(i);
    total_bad_meas_before = length(find(bad_meas_idxs <= curr_max_idx));
    curr_max_idx = curr_max_idx + total_bad_meas_before;
    while ismember(curr_max_idx, bad_meas_idxs)
        curr_max_idx = curr_max_idx + 1;
    end
    shifted_idxs(i) = curr_max_idx;
end