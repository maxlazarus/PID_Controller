A = csvread('PD880Hz.csv');
A = ((3000 - A) / 6000) + 1;

B = csvread('P880Hz.csv');
B = ((3000 - B) / 6000) + 1;

interval = 1/880;
t = 0:interval:1;
t = t(1:880);

G = tf([1000],[1 18 1]);
G = G / (1 + G);
C = step(G, t);

mu = 32;
phi = 0.8;
rho = 30;
epsilon = 10;
delay = 0;
for i = 1:numel(t)
    if(t(i) < delay)
        D(i) = 0;
    else
        tnow = t(i) - delay;
        ... (1 - exp(-rho*tnow)*sin(mu*tnow + phi)/sin(phi))
        ... (1 - exp(-rho*tnow))*
        D(i) = (1 - exp(-rho*tnow))*(1 +  1.5 * exp(-epsilon*tnow) * sin(mu*tnow + phi));
    end
end

linear = 5.9;
for i = 1:numel(t)
    if(t(i) < 1 / linear)
        E(i) = linear * t(i);
    else
        E(i) = 1;
    end
end

clf;
whitebg('black');
set(gca,'Xcolor',[0.5 0.5 0.5]);
set(gca,'Ycolor',[0.5 0.5 0.5]);
hold on;
grid on;

... plot(t, A, 'red');
... plot(t, C, 'green');
... plot(t, D, 'Color', [0.5, 0, 1]);
... plot(t, E, 'blue');
plot(t, B, 'white');