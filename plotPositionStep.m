variables;
[Gm, G, H] = createSystem(K_m, T_m, w_max_unloaded, 0);

clf;
numFiles = 5;
fileNames = cell(numFiles, 1);
fileNames{1} = ['P2positionStep.csv'];
fileNames{2} = ['P1positionStep.csv'];
fileNames{3} = ['P0_75positionStep.csv'];
fileNames{4} = ['P0_5positionStep.csv'];
fileNames{5} = ['P0_2positionStep.csv'];

    
PIDvalues = [[2, 0, 0]', [1, 0, 0]', [0.75, 0, 0]', [0.5, 0, 0]', [0.2, 0, 0]'];

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

    setGraphStyle(strcat('UNLOADED, P =', num2str(PIDvalues(1, i))));
end