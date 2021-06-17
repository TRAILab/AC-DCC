function shifted_idxs = getModifiedIdxs(bad_meas_idxs, max_idxs)

shifted_idxs = zeros(length(max_idxs),1);

for i=1:length(max_idxs)
    shifted_idxs(i) = max_idxs(i) + sum(bad_meas_idxs<max_idxs(i));
end