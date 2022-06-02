I = 0.2517;  % kg-m^2
wn = 0.5;  % desired natural frequency
zeta = 1.5;  % desired damping ratio

Kp = I*wn^2
Kd = 0.75*I*2*zeta*wn