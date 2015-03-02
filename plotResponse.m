variables;
[G_m, G, H] = createSystem(K_m, T_m, w_max, J);
x = normalize(fileName);
v = d(x);

dt = 1 / controlFrequency; % seconds
t = (0:dt:0.999999);

clf;
hold on;

% measured step response

plot(t, x, '.', 'MarkerSize', 5);
% plot(t, v, '.');%'MarkerSize', 5);

% system model step response

H_m = step(t, G_m);
H = step(t, G);

% plot(t, H_m, '--', 'LineWidth', 1);
plot(t, H, '--', 'LineWidth', 1);

legend('Measured position (rad)', 'Modelled position (rad)');

setGraphStyle(titleText);