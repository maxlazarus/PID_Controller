% from data sheet:

max_torque = 0.0765; % Nm
V_nom = 12; % 12 V
w_0 = 20.9; % 200 RPM = 200/60 RPS = 2pi*200/60 rad/s = 20.9 rad/s
w_loaded = 17.1; % 163 RPM = 17.1 rad/s
K_m = Vnom / w_0; % Nm/A

% measured / calculated

nut_m = 0.075; % kg
nut_r = 0.30; % m
J_nut = nut_m * nut_r^2 / 2;

l_bar_m = 0.074; % kg
l_bar_r = 0.31; % m
J_bar = l_bar_m * l_bar_r^2 / 3;

w_max_long_bar = 25.1; % rad/s, from v step with long bar
w_max_unloaded = 25.7; % rad/s, from v step
% T_m = J/B = dt from w = 0 to w = (0.6321 * w_Max)
T_m = 0.065; % s

w_max = w_max_long_bar;

J_m = K_m / w_max * T_m; 
B_m = K_m / w_max;

motorDynamics = tf([K_m], [(J_bar + J_m) B_m]);
positionSensor = tf([1], [1 0]);
G = motorDynamics * positionSensor;
H = 1;