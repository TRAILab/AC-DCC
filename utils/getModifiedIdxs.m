function shifted_idxs = getModifiedIdxs(total_meas, bad_meas_idxs, max_idxs)

shifted_idxs = zeros(length(max_idxs),1);
good_idxs = setdiff(1:total_meas, bad_meas_idxs);

for i=1:length(max_idxs)
     new_idx = max_idxs(i) + sum(bad_meas_idxs<max_idxs(i));
     temp = good_idxs(good_idxs>new_idx);
     shifted_idxs(i) = temp(1); % Select the first value
end