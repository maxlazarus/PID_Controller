% x = normalize('pos500.csv');
x = normalize('long_bar.csv', 1);

v = d(x);
% a = dSmooth(v, 1);

dt = 1 / controlFrequency; % seconds
t = (0:dt:0.999999);

setGraphStyle();

plot(t, x);
plot(t, v);

transferFunctions;
step(t, motorDynamics);
step(t, G);

setGraphStyle();