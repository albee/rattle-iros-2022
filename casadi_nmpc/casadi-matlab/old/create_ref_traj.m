%{
Create a reference trajectory to follow.
Currently produces up to 2nd derivative for double integrator systems.

dt - timestep
tf - final time
DIMS - system name
REF_TRAJ - ref traj name
%}
function ref_traj = create_ref_traj(dt, tf, DIMS, REF_TRAJ)
    num_setpoints = ceil(tf/dt) + 1;  % same as N---inclusive of start and end [t0; ...; tN]
    t_des_hist = linspace(0,tf,num_setpoints)';
            
    switch DIMS
        case '1D'
            switch REF_TRAJ
                case 'SINUSOID'
                    samples1 = linspace(0,2*pi*3, num_setpoints);
                    x = 0.3*sin(samples1)';
                    
                    x_des_hist = [x];
                    
                    xd_des_hist = [0; diff(x_des_hist)]/dt;
                    
                    x_des_hist = [x_des_hist, xd_des_hist];
                    
                    xdd_des_hist = [0; diff(xd_des_hist)]/dt;  % accelerations

                case 'STEP'
                    x_des_hist = [zeros(ceil(num_setpoints*.1),1); ones(floor(num_setpoints*.9),1)];

                    xd_des_hist = [0; diff(x_des_hist)]/dt;

                    x_des_hist = [x_des_hist, xd_des_hist];

                    xdd_des_hist = [0; diff(xd_des_hist)]/dt;  % accelerations
            end
            
        case '3D'
            switch REF_TRAJ
                case 'SINUSOID'
                    samples1 = linspace(0,2*pi, num_setpoints);
                    x = [0.3*sin(samples1)', 0.2*sin(samples1)', 0.1*sin(samples1)'];  % positions to follow

                    x_des_hist = [x];

                    xd_des_hist = [[0; diff(x_des_hist(:,1))] [0; diff(x_des_hist(:,2))] [0; diff(x_des_hist(:,3))]]/dt;

                    x_des_hist = [x_des_hist, xd_des_hist];

                    xdd_des_hist = [[0; diff(xd_des_hist(:,1))] [0; diff(xd_des_hist(:,2))] [0; diff(xd_des_hist(:,3))]]/dt;  % accelerations
                case 'STEP'  % TODO
                    x_des_hist = [zeros(ceil(num_setpoints*.1),1); ones(floor(num_setpoints*.9),1)];

                    xd_des_hist = [0; diff(x_des_hist)]/dt;

                    x_des_hist = [x_des_hist, xd_des_hist];

                    xdd_des_hist = [0; diff(xd_des_hist)]/dt;  % accelerations
                    
                case 'TEST'  % TODO
                    x_des_hist = zeros(num_setpoints, 6);
                    xdd_des_hist = zeros(num_setpoints, 3);  % accelerations
            end
    end

    ref_traj = [t_des_hist, x_des_hist xdd_des_hist];
end