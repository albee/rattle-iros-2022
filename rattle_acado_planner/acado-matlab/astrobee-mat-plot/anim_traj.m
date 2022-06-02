%{
Animate a trajectory in ROAM/ReSWARM state vector format.
%}
function anim_traj(traj)
    x = traj(:, 2:4);
    quat = traj(:, 8:11);
    anim_tumble(x, quat, x);
end