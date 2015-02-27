function [ x ] = normalize(filename, seconds)

    controlFrequency = 500; % Hz
    length = controlFrequency * seconds;
    ticksPerRadian = 12000 / (2 * pi);
    
    x = csvread(filename);
    x = x / ticksPerRadian;
    x = x(1:length);
    startingPosition = min(x);
    x = x - startingPosition;
end

