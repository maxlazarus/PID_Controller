function [dx] = d(x)
    size = numel(x);
    dx = zeros(1, size);
    for i = 2:size;
        dx(i) = (x(i) - x(i - 1));
    end
    dx(1) = 0;
end

