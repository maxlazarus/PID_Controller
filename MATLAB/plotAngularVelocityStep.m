x = csvread('angularV.csv');
x = x(:,2);
for i = 1:size(x)
    if(x(i) == 0)
        x(i) = 8192;
    end
end
x = x(1:600);
t = 0:0.004:(0.004 * 599);
x = -x;
minX = min(x);
x = (x - minX);
x = (x / 4096) * 3.1415926;

plot(t, x)
% v = dSmooth(x, 5);
% plot(v);
% hold on;
% plot(x);
