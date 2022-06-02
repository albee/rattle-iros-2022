function [input] = wrench2input(wrench)
    input = [wrench.Force.X, wrench.Force.Y, wrench.Force.Z, ...
     wrench.Torque.X, wrench.Torque.Y, wrench.Torque.Z];
end