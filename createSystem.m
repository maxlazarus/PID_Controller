function [G_m, G, H] = createSystem(K_m, T_m, w_max, J_added)

    J_m = K_m / w_max * T_m; 
    B_m = K_m / w_max;   
    
    positionSensor = tf([1], [1 0]); % rotary encoder
    G_m = tf([K_m], [(J_m + J_added) B_m]); % motor
    G = G_m * positionSensor; % open-loop system
    H = 1;
end

