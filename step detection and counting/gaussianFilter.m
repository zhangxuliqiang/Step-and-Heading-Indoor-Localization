function window = gaussianFilter(window_size, std)
%GAUSSIAN FILTER: Summary of this function goes here
%   Detailed explanation goes here

temp_window = zeros(1,window_size);

for n = 0:1:window_size-1
    value = exp(-0.5 * ((n - (window_size - 1) / 2) / ...
        (std * (window_size - 1) / 2))^2);
    temp_window(1,n+1) = value;
end

window = temp_window;
end

