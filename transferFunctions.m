% from data sheet:
Vnom = 12; % 12 V
w0 = 20.9; % 200 RPM = 200/60 RPS = 2pi*200/60 rad/s = 20.9 rad/s
Km = Vnom / w0; % Nm/A
K = 50;
Bmotor = 2.5;
Jmotor = .15;
motorDynamics = tf([Km], [Jmotor Bmotor])
positionSensor = tf([1], [1 0]);
G = K * motorDynamics * positionSensor;
H = 1;
system = G / (G * H + 1);

plot880
step(system)