function [dx] = d(x, average)
    size = numel(x);
    dx = zeros(1, size);
    for i = average + 1:size;
        dx(i) = size * (x(i) - x(i - average)) / average;
    end
    for i = 1: (average - 1);
        dx(i) = dx(average - i + 1) / (average - i); 
    end
end

