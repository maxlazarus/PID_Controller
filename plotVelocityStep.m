variables;

fileName = 'long_bar.csv';
w_max = w_max_long_bar;
J = J_long_bar;
titleText = 'VELOCITY STEP RESPONSE - LONG BAR';

plotResponse; % variables above passed implicitly
figure;

fileName = 'velocityShortLoaded.csv';
w_max = w_max_unloaded;
J = J_short_bar + J_nut;
titleText = 'VELOCITY STEP RESPONSE - SHORT BAR WITH NUT';

plotResponse; % variables above passed implicitly
figure;

fileName = 'pos500.csv';
w_max = w_max_unloaded;
J = 0;
titleText = 'VELOCITY STEP RESPONSE - UNLOADED';

plotResponse; % variables above passed implicitly