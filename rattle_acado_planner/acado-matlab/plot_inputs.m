function plot_inputs(u_plan)
  %{
  Plot the system inputs (force, torque).

  @param u_plan: (Nx6)
  %}

  figure;
  hold on;
  subplot(2, 3, 1)
  plot(u_plan(:,1));
  title('$F_x$');
  
  subplot(2, 3, 2)
  plot(u_plan(:,2));
  title('$F_y$');
  
  subplot(2, 3, 3)
  plot(u_plan(:,3));
  title('$F_z$');
  
  subplot(2, 3, 4)
  plot(u_plan(:,4));
  title('$\tau_x$');
  
  subplot(2, 3, 5)
  plot(u_plan(:,5));
  title('$\tau_y$');
  
  subplot(2, 3, 6)
  plot(u_plan(:,6));
  title('$\tau_z$');
end