% from data sheet:

max_torque = 0.0765; % Nm
V_nom = 12; % 12 V
w_0 = 20.9; % 200 RPM = 200/60 RPS = 2pi*200/60 rad/s = 20.9 rad/s
w_loaded = 17.1; % 163 RPM = 17.1 rad/s
K_m = V_nom / w_0; % Nm/A

% measured / calculated

nut_m = 0.075; % kg
nut_r = 0.115; % m, placement of nut on bar
J_nut = nut_m * nut_r^2;

short_bar_m = 0.033; % kg
short_bar_r = 0.115; % m
J_short_bar = short_bar_m * short_bar_r^2 / 3;

long_bar_m = 0.074; % kg
long_bar_r = 0.31; % m
J_long_bar = long_bar_m * long_bar_r^2 / 3;

w_max_long_bar = 25.1; % rad/s, from v step with long bar
w_max_unloaded = 25.7; % rad/s, from v step
% T_m = J/B = dt from w = 0 to w = (0.6321 * w_Max)
T_m = 0.065; % s

% time index

controlFrequency = 500;
dt = 1 / controlFrequency;
t = 0 : dt : 0.9999;




