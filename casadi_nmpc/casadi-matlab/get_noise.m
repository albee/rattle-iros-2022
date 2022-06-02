function [noise, noise_mpc_dt] = get_noise(time_factor)
    %{
    Set noise levels. This is the noise at control_dt!
    
    @return:
    noise: system noise at control_dt struct of [r1 r2 r3 v1 v2 v3]
    noise_mpc_dt: system noise at mpc_dt
    %}
%     noise.r1 = 0.00;  % disturbance per control_dt
%     noise.r2 = 0.00;
%     noise.r3 = 0.00;
%     noise.v1 = 0.00;
%     noise.v2 = 0.00;
%     noise.v3 = 0.00;
 
%     noise.r1 = 0.005;  % disturbance per control_dt
%     noise.r2 = 0.005;
%     noise.r3 = 0.005;
%     noise.v1 = 0.0025;
%     noise.v2 = 0.0025;
%     noise.v3 = 0.0025;
    
    noise.r1 = 0.0075;  % disturbance per control_dt
    noise.r2 = 0.0075;
    noise.r3 = 0.0075;
    noise.v1 = 0.01;
    noise.v2 = 0.01;
    noise.v3 = 0.01;

%     noise.r1 = 0.0331;  % disturbance per control_dt
%     noise.r2 = 0.0260;
%     noise.r3 = 0.0537;
%     noise.v1 = 0.0069;
%     noise.v2 = 0.0055;
%     noise.v3 = 0.0073;
    
    noise_mpc_dt.r1 = noise.r1*time_factor;  % disturbance per control_dt
    noise_mpc_dt.r2 = noise.r2*time_factor;
    noise_mpc_dt.r3 = noise.r3*time_factor;
    noise_mpc_dt.v1 = noise.v1*time_factor;
    noise_mpc_dt.v2 = noise.v2*time_factor;
    noise_mpc_dt.v3 = noise.v3*time_factor;
    
    
end