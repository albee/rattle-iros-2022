clear;
close all;
clc;
load('rrtlqrfulldyn.mat');
% load('NCOptwithManipulation.mat');
load('traj_smoother.mat');
r=1./sqrt(eig(solution.P(:,:,1)));
direction = [0 0 1];
i=0;
k_iter=1;
dt=solution.step_size*solution.DT;
% t=linspace(0,length(traj)*dt,length(traj));
% x_interp=zeros(13,length(t));
% x_interp(:,1)=traj(:,1);
% for k=1:length(t)-1
%     %x_interp(:,k+1)=x_interp(:,k)+dt*fn_dyn(x_interp(:,k),u_traj(:,k),solution.m,solution.I);
%     f1=dt*fn_dyn(x_interp(:,k),u_traj(:,k),solution.m,solution.I);
%     f2=dt*fn_dyn(x_interp(:,k)+.5*f1,u_traj(:,k),solution.m,solution.I);
%     f3=dt*fn_dyn(x_interp(:,k)+.5*f2,u_traj(:,k),solution.m,solution.I);
%     f4=dt*fn_dyn(x_interp(:,k)+f3,u_traj(:,k),solution.m,solution.I);
%     x_interp(:,k+1)=x_interp(:,k)+1/6*(f1+2*f2+2*f3+f4);
%     x_interp(7:10,k+1) =x_interp(7:10,k+1)/norm(x_interp(7:10,k+1));
% end
% x_interp=interp1(t_sm,x_sm',time)';
figure
subplot(4,1,1)
plot(traj(7,:),'LineWidth',2);
hold on;
% plot(x_interp(7,:),'-.r','LineWidth',2);
% plot(x_sm(7,:),'-.g','LineWidth',2);
plot(xlqr_sm(7,:),'-.m','LineWidth',2);
ylabel('q1');
subplot(4,1,2)
plot(traj(8,:),'LineWidth',2);
hold on;
% plot(x_interp(8,:),'-.r','LineWidth',2);
% plot(x_sm(8,:),'-.g','LineWidth',2);
plot(xlqr_sm(8,:),'-.m','LineWidth',2);
ylabel('q2');
subplot(4,1,3)
plot(traj(9,:),'LineWidth',2);
hold on;
% plot(x_interp(9,:),'-.r','LineWidth',2);
% plot(x_sm(9,:),'-.g','LineWidth',2);
plot(xlqr_sm(9,:),'-.m','LineWidth',2);
ylabel('q3');
subplot(4,1,4)
plot(traj(10,:),'LineWidth',2);
hold on;
% plot(x_interp(10,:),'-.r','LineWidth',2);
% plot(x_sm(10,:),'-.g','LineWidth',2);
plot(xlqr_sm(10,:),'-.m','LineWidth',2);
legend('RRT','nonconvex','smoothed LQR','Location','southeast');
ylabel('q4');

figure
subplot(3,1,1)
plot(traj(4,:),'LineWidth',2);
hold on;
% plot(x_interp(7,:),'-.r','LineWidth',2);
% plot(x_sm(4,:),'-.g','LineWidth',2);
plot(xlqr_sm(4,:),'-.m','LineWidth',2);
ylabel('xdot');
subplot(3,1,2)
plot(traj(5,:),'LineWidth',2);
hold on;
% plot(x_interp(5,:),'-.r','LineWidth',2);
% plot(x_sm(5,:),'-.g','LineWidth',2);
plot(xlqr_sm(5,:),'-.m','LineWidth',2);
ylabel('ydot');
subplot(3,1,3)
plot(traj(6,:),'LineWidth',2);
hold on;
% plot(x_interp(6,:),'-.r','LineWidth',2);
% plot(x_sm(6,:),'-.g','LineWidth',2);
plot(xlqr_sm(6,:),'-.m','LineWidth',2);
ylabel('zdot');

figure
plot3(traj(1,:),traj(2,:),traj(3,:),'LineWidth',2)
hold on;
% plot3(x_interp(1,:),x_interp(2,:),x_interp(3,:),'-r','LineWidth',2)
plot3(solution.end.sv(1),solution.end.sv(2),solution.end.sv(3),'xr','LineWidth',2);
plot3(solution.start.sv(1),solution.start.sv(1),solution.start.sv(1),'*g','LineWidth',2);
%plot3(x_opt(1,:),x_opt(2,:),x_opt(3,:),'-c','LineWidth',2);
plot3(x_sm(1,:),x_sm(2,:),x_sm(3,:),'-g','LineWidth',2);
%plot3(xlqr_sm(1,:),xlqr_sm(2,:),xlqr_sm(3,:),'-m','LineWidth',2);
plot3(traj_lqrlqrv2(1,:),traj_lqrlqrv2(2,:),traj_lqrlqrv2(3,:),'-.c','LineWidth',2);
xlabel('x');
ylabel('y');
legend('RRT','target','start','line Interp','LQR Interp','Location','southeast');
% xlim([.5 1.1]);
% ylim([.2 1.1]);
axis('square')
grid on;
for k=1:size(solution.obstacle_list,2)
%     circle2(zobs(1,k),zobs(2,k),double(robs))
    ellipsoid(solution.obstacle_list(1,k),solution.obstacle_list(2,k),solution.obstacle_list(3,k),r(1),r(2),r(3))
end
hold off;
% plot3DD(xlqr_sm,solution.obstacle_list,r)
%set(0, 'DefaultFigureVisible', 'off');


figure
subplot(4,3,1)
hold on;
plot(traj_lqrlqrv2(1,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(1,:),'-.k','LineWidth',2);
ylabel('x');
subplot(4,3,2)
hold on;
plot(traj_lqrlqrv2(2,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(2,:),'-.k','LineWidth',2);
ylabel('y');
subplot(4,3,3)
hold on;
plot(traj_lqrlqrv2(3,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(3,:),'-.k','LineWidth',2);
ylabel('z');

subplot(4,3,4)
hold on;
plot(traj_lqrlqrv2(4,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(4,:),'-.k','LineWidth',2);
ylabel('xdot');
subplot(4,3,5)
hold on;
plot(traj_lqrlqrv2(5,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(5,:),'-.k','LineWidth',2);
ylabel('ydot');
subplot(4,3,6)
hold on;
plot(traj_lqrlqrv2(6,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(6,:),'-.k','LineWidth',2);
ylabel('zdot');

subplot(4,3,7)
hold on;
plot(traj_lqrlqrv2(7,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(7,:),'-.k','LineWidth',2);
ylabel('q1');
subplot(4,3,8)
hold on;
plot(traj_lqrlqrv2(8,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(8,:),'-.k','LineWidth',2);
ylabel('q2');
subplot(4,3,9)
hold on;
plot(traj_lqrlqrv2(9,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(9,:),'-.k','LineWidth',2);
ylabel('q3');

subplot(4,3,10)
hold on;
plot(traj_lqrlqrv2(11,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(11,:),'-.k','LineWidth',2);
ylabel('q1dot');
subplot(4,3,11)
hold on;
plot(traj_lqrlqrv2(12,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(12,:),'-.k','LineWidth',2);
ylabel('q2dot');
subplot(4,3,12)
hold on;
plot(traj_lqrlqrv2(13,:),'-.c','LineWidth',2);
plot(xref_lqrlqrv2(13,:),'-.k','LineWidth',2);
ylabel('q3dot');

figure(25);
plot3(solution.start.x,solution.start.y,solution.start.z,'*g','LineWidth',2);
hold on;
plot3(solution.end.x,solution.end.y,solution.end.z,'xr','LineWidth',2);
xlim([-2 12]);
ylim([-2 12]);
zlim([-2 12]);

xlabel('x');
ylabel('y');
zlabel('z');
grid on;
for k=1:size(solution.obstacle_list,2)
%     circle2(zobs(1,k),zobs(2,k),double(robs))
    ellipsoid(solution.obstacle_list(1,k),solution.obstacle_list(2,k),solution.obstacle_list(3,k),r(1),r(2),r(3))
end
for node=solution.node_list
    if ~isempty(node{1}.parent)
       plot3(node{1}.path_x,node{1}.path_y,node{1}.path_z,'-g','LineWidth',2) 
       view(gca,i,0)
       i=i+4;
       
    end
    mm(:,k_iter) = getframe(gcf);
       k_iter=k_iter+1;
    

    %pause(.3);
end
plot3(traj(1,:),traj(2,:),traj(3,:),'--b','LineWidth',2)
%     if isa(rnd,'Node')
%         plot(rnd.x,rnd.y,'^k')
%     end
for j=1:90
    view(gca,i,0)
    i=i+4;
    mm(k_iter) = getframe(gcf);
    k_iter=k_iter+1;
end
    
hold off; 
v = VideoWriter('ggg1.gif');
v.FrameRate=50;
open(v);
writeVideo(v,mm);
close(v);

%plot3DD(traj,zobs,r);

function R=quat2Rot(q)
R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2 2*(q(1)*q(2)+q(3)*q(4)) 2*(q(1)*q(3)-q(2)*q(4));...
   2*(q(1)*q(2)-q(3)*q(4)) -q(1)^2+q(2)^2-q(3)^2+q(4)^2 2*(q(2)*q(3)+q(1)*q(4));...
   2*(q(1)*q(3)+q(2)*q(4)) 2*(q(2)*q(3)-q(1)*q(4)) -q(1)^2-q(2)^2+q(3)^2+q(4)^2];
end

function plot3DD(x,zobs,r)
    figure;
    hold on;
    xlim([-2 8]);
    ylim([-2 8]);
    zlim([-2 8]);
    axis('square');
    grid on;
    view(gca,45,45)
    for k=1:size(zobs,2)
        ellipsoid(zobs(1,k),zobs(2,k),zobs(3,k),r(1)*2/3,r(2)*2/3,r(3)*2/3)
    end
    count=1;
    for k=1:10:length(x)
        R=quat2Rot(x(7:10,k));
        CLR=[0 0 1];
        ALPHA=.8;
        l=.5;
        [qq q1 q2 q3]=plot_cube(x(1:3,k),R,l,CLR,ALPHA);
        mm(:,count) = getframe(gcf);
        count=count+1;
        pause(.1)
        if k~=length(x)
           delete(qq);
           delete(q1);
           delete(q2);
           delete(q3);
        end
    end
    v = VideoWriter('ggg2.gif');
    v.FrameRate=50;
    open(v);
    writeVideo(v,mm);
    close(v);

end

function xdot=fn_dyn(x,u,m,I)
    xyzdot=x(4:6);
    xyzdotdot=u(1:3)/m;
    qdot=.5*[0 x(13) -x(12) x(11); -x(13) 0 x(11) x(12); x(12) -x(11) 0 x(13); -x(11) -x(12) -x(13) 0]*x(7:10);
    wdot=inv(I)*(cross(-x(11:13),I*x(11:13)))+inv(I)*u(4:6);
    xdot=[xyzdot;xyzdotdot; qdot; wdot];

end
