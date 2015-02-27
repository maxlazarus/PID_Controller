function [ x ] = normalize(filename)

    controlFrequency = 500; % Hz
    length = controlFrequency;
    ticksPerRadian = 12000 / (2 * pi);
    
    x = csvread(filename);
    x = x / ticksPerRadian;
    x = x(1:length);
    startingPosition = min(x);
    x = x - startingPosition;
end

