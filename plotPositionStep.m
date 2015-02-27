transferFunctions;

clf;

figure(1);
x = normalize('P0_2positionStep.csv');
plot(t, x);
setGraphStyle();
PID = pid(0.2, 0, 0);
system = PID * G / (PID * G * H + 1);
step(system);
setGraphStyle();

figure(2);
x = normalize('P0_5positionStep.csv');
plot(t, x);
setGraphStyle();
PID = pid(0.5, 0, 0);
system = PID * G / (PID * G * H + 1);
step(system);
setGraphStyle();

figure(3);
x = normalize('P0_75positionStep.csv');
plot(t, x);
setGraphStyle();
PID = pid(0.75, 0, 0);
system = PID * G / (PID * G * H + 1);
step(system);
setGraphStyle();

figure(4);
setGraphStyle();
x = normalize('P1positionStep.csv');
plot(t, x);
PID = pid(1, 0, 0);
system = PID * G / (PID * G * H + 1);
step(system);
setGraphStyle();

figure(5);
setGraphStyle();
x = normalize('P2positionStep.csv');
plot(t, x);
PID = pid(2, 0, 0);
system = PID * G / (PID * G * H + 1);
step(system);
setGraphStyle();