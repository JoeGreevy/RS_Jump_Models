function [best_mean, best_std, best_idx] = quiet_avg(x, winsize, stride)
%QUIET_AVG Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    x
    winsize = 200
    stride = 100
end

arguments (Output)
    best_mean
    best_std
    best_idx
end
n = length(x);
num_windows = floor((n - winsize) / stride) + 1;
best_mean = NaN;
best_std  = Inf;
best_idx  = NaN;
starts = 1:stride:length(x)-winsize+1;

for i = starts
    win    = x(i:i+winsize-1);

    if sum(isnan(win)) > 0.5 * winsize
        continue  % skip windows with too many NaNs
    end

    win_std = std(win, 0, 'omitnan');  % flag 0 = normalise by N-1
    if win_std < best_std
        best_std  = win_std;
        best_mean = mean(win, 'omitnan');
        best_idx  = i;
    end
end


end