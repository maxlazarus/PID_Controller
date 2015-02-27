variables;
[Gm, G, H] = createSystem(K_m, T_m, w_max_unloaded, 0);

clf;
numFiles = 5;
fileNames = cell(numFiles, 1);
fileNames{1} = ['P0_44I0_068D0_033positionStepShortLoaded.csv'];
fileNames{2} = ['P0_552D0_045positionStepShortLoaded.csv'];
fileNames{3} = ['P0_4I0_04D0_2positionStepShortLoaded.csv'];
fileNames{4} = ['P0_3I0_035D0_015positionStepShortLoaded.csv'];
fileNames{5} = ['P0_27D0_017positionStepShortLoaded.csv'];

    
PIDvalues = [[0.44, 0.068, 0.033]', [0.552, 0, 0.045]', [0.4, 0.04, 0.2]', [0.3, 0.035, 0.01]', [0.27, 0, 0.017]'];

for i = 1:numFiles
    % disp(('Hello I am the number ',int2str(i)));
    if (i > 1)
        figure;
    end
  
    x = normalize(fileNames{i});
    plot(t, x);

    hold on;
    
    PID = pid(PIDvalues(1, i), PIDvalues(2, i), PIDvalues(3, i));
    system = PID * G / (PID * G * H + 1);
    step(system);

    s = '';
    s = strcat(s, 'SHORT BAR WITH NUT P =', num2str(PIDvalues(1, i)));
    s = strcat(s, ' I =', num2str(PIDvalues(2, i)));
    s = strcat(s, ' D =', num2str(PIDvalues(3, i)));
    setGraphStyle(s);
end