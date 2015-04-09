s = csvread('angularPositionStep.csv');
s = s(:,2)
s = s(1:1000);
t = 0:0.004:(0.004 * 999);
minX = min(s);
s = (s - minX);
s = (s / 4096) * 180;
plot(t, s)