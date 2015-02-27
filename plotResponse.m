variables;
[G_m, G, H] = createSystem(K_m, T_m, w_max, J);
x = normalize(fileName);
v = d(x);

dt = 1 / controlFrequency; % seconds
t = (0:dt:0.999999);

clf;
hold on;

% measured step response

plot(t, x);
plot(t, v);

% system model step response

step(t, G_m);
step(t, G);

setGraphStyle(titleText);