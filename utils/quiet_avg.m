function [avg, dev, ind] = quiet_avg(x, winsize, stride)
%QUIET_AVG Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    x
    winsize = 200
    stride = 100
end

arguments (Output)
    avg
    dev
    ind
end
n = length(x);
num_windows = floor((n - winsize) / stride) + 1;
win_stds  = zeros(1, num_windows);
win_means = zeros(1, num_windows);

for i = 1:num_windows
    start_idx = (i - 1) * stride + 1;
    end_idx   = start_idx + winsize - 1;
    window    = x(start_idx:end_idx);

    win_stds(i)  = std(window);
    win_means(i) = mean(window);
end
[dev, ind] = min(win_stds);
avg      = win_means(ind);
% be careful with the matlab index

end