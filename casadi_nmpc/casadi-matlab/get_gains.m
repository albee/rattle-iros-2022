function [gains_mpc, gains_dr] = get_gains(USE_TS1)
    %{
    Set controller gains for MPC and Tube MPC.
    %}
    if USE_TS1 == 0
        gains_mpc.Q1 = 50;
        gains_mpc.Q2 = 50;
        gains_mpc.Q3 = 50;
        gains_mpc.Q4 = 5;
        gains_mpc.Q5 = 5;
        gains_mpc.Q6 = 5;
        gains_mpc.R1 = 0.1;
        gains_mpc.R2 = 0.1;
        gains_mpc.R3 = 0.1;
        gains_mpc.QN1 = 100;
        gains_mpc.QN2 = 100;
        gains_mpc.QN3 = 100;
        gains_mpc.QN4 = 10;
        gains_mpc.QN5 = 10;
        gains_mpc.QN6 = 10;

        gains_dr.Q1 = 5;
        gains_dr.Q2 = 5;
        gains_dr.Q3 = 5;
        gains_dr.Q4 = 10;
        gains_dr.Q5 = 10;
        gains_dr.Q6 = 10;
        gains_dr.R1 = 2;
        gains_dr.R2 = 2;
        gains_dr.R3 = 2;
    else
        % ISS-TS1 values
        weights
        m = 8.0;  % mass (ground units), [kg]
        gains.Q1 = 50;
        gains.Q2 = 50;
        gains.Q3 = 50;
        gains.Q4 = 5;
        gains.Q5 = 5;
        gains.Q6 = 5;
        gains.R1 = 11;
        gains.R2 = 11;
        gains.R3 = 11;
        gains.QN1 = 100;
        gains.QN2 = 100;
        gains.QN3 = 100;
        gains.QN4 = 100;
        gains.QN5 = 100;
        gains.QN6 = 100;
        
        gains_dr.Q1 = 5;
        gains_dr.Q2 = 5;
        gains_dr.Q3 = 5;
        gains_dr.Q4 = 50;
        gains_dr.Q5 = 50;
        gains_dr.Q6 = 50;
        gains_dr.R1 = 1;
        gains_dr.R2 = 1;
        gains_dr.R3 = 1;
    end
end