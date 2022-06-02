function x_lqrrrt = plot_lqr_rrt(rrt_traj_msgs)
  N_lqrrrt=length(rrt_traj_msgs{1}.Data)/20;
    x_lqrrrt=zeros(13,N_lqrrrt);
    t_lqrrrt=0:.2:(N_lqrrrt-1)*.2;

    for k=1:N_lqrrrt
       x_lqrrrt(1,k)= rrt_traj_msgs{1}.Data(20*k-18);
       x_lqrrrt(2,k)= rrt_traj_msgs{1}.Data(20*k-17);
       x_lqrrrt(3,k)= rrt_traj_msgs{1}.Data(20*k-16);

       x_lqrrrt(4,k)= rrt_traj_msgs{1}.Data(20*k-11);
       x_lqrrrt(5,k)= rrt_traj_msgs{1}.Data(20*k-10);
       x_lqrrrt(6,k)= rrt_traj_msgs{1}.Data(20*k-9);

       x_lqrrrt(7,k)= rrt_traj_msgs{1}.Data(20*k-15);
       x_lqrrrt(8,k)= rrt_traj_msgs{1}.Data(20*k-14);
       x_lqrrrt(9,k)= rrt_traj_msgs{1}.Data(20*k-13);
       x_lqrrrt(10,k)= rrt_traj_msgs{1}.Data(20*k-12);

       x_lqrrrt(11,k)= rrt_traj_msgs{1}.Data(20*k-8);
       x_lqrrrt(12,k)= rrt_traj_msgs{1}.Data(20*k-7);
       x_lqrrrt(13,k)= rrt_traj_msgs{1}.Data(20*k-6);

    end
    x_lqrrrt
    
    figure;
    title('hi')
    plot(x_lqrrrt(6,:))
end