function state = plot_gnc_ekf(ekf_msgs)
    %{ Plot gnc/ekf using 3D animation tool
    %}
    state = [];  % x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
    for i = 1:1:length(ekf_msgs)
        state(i, :) = ekf2state(ekf_msgs{i});
    end
    
    % Visualizes 3D,
    % x = matrix of [x, y, z] position
    % quat = matrix of [qx, qy, qz, qw] quaternions
    state_rt = state(1:62.5:end, :);  % once every 62.5 periods (1s)
    x = state_rt(:, 1:3);
    quat = state_rt(:, 7:10);
    anim_tumble(x, quat, x);
end