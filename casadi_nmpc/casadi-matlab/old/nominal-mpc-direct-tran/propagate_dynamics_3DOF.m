    %% Dynamics
    % Run the nominal forward dynamics MODEL, do NOT update state
    function [xk1, zk1] = propagate_dynamics_3DOF(dt, mass, xk, uk)
        m1 = mass;
        
        A = [1, 0, 0, dt, 0, 0;
          0, 1, 0, 0, dt, 0;
          0, 0, 1, 0, 0, dt;
          0, 0, 0, 1, 0, 0;
          0, 0, 0, 0, 1, 0;
          0, 0, 0, 0, 0, 1];

        B = [dt^2/(2*m1),   0,               0;
                  0,               dt^2/(2*m1),   0;
                  0,               0,               dt^2/(2*m1);
                  dt/m1, 0,               0;
                  0,               dt/m1, 0;
                  0,               0,               dt/m1];
    
        C = eye(6);
        
        xk1 = A*xk + B*uk;
        zk1 = C*xk;
    end