function plot_track_and_traj(track, traj)
    %{
    Plot track and des traj
    %}
    figure;
    sgtitle('Localization Position History', 'FontSize', 40);
    
    %3D
    subplot(2,2,1);
    hold on;
    plot3(traj(:, 2), traj(:, 3), traj(:, 4));
    plot3(track(:, 1), track(:, 2), track(:, 3), 'black');
    set(gca,'zdir','reverse', 'xdir','reverse')
    grid on;
    axis equal;
    view(215, 45);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    
    % xz, PORT
    subplot(2,2,2);
    hold on;
    plot(track(:, 1), track(:, 3));
    
    title('$xz$ view, PORT')
    xlabel('x [m]');
    ylabel('z [m]');
    set(gca, 'ydir','reverse')
    grid on;
    axis equal;
    
    % xy, DECK
    subplot(2,2,3);
    hold on;
    plot(track(:, 1), track(:, 2));
   
    title('$xy$ view, DECK')
    xlabel('x [m]');
    ylabel('y [m]');
    set(gca,'ydir','reverse')
    grid on;
    axis equal;
    
    % yz, FWD
    subplot(2,2,4);
    hold on;
    plot(track(:, 2), track(:, 3));
    
    title('$yz$ view, FWD')
    xlabel('y [m]');
    ylabel('z [m]');
    set(gca,'ydir','reverse')
    grid on;
    axis equal;
end