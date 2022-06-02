% Set dynamics based on dt zero-order hold
function [A, B] = get_dynamics_for_dt(dt, m1)
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
end
