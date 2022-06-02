close all;

figure()
subplot(2,3,1)
hold on
plot(time_store, x_des(1)*ones(length(time_store), 1), 'black')
plot(time_store, states(:,1), 'r')
title('x');

subplot(2,3,2)
hold on
plot(time_store, x_des(2)*ones(length(time_store), 1), 'black')
plot(time_store, states(:,2), 'r')
title('y');

subplot(2,3,3)
hold on
plot(time_store, x_des(3)*ones(length(time_store), 1), 'black')
plot(time_store, states(:,3), 'r')
title('z');
X = states(:,1);
Y = states(:,2);
Z = states(:,3);

subplot(2,3,4)
hold on
plot(time_store, x_des(4)*ones(length(time_store), 1), 'black')
plot(time_store, states(:,4), 'r')
title('Vx');

subplot(2,3,5)
hold on
plot(time_store, x_des(5)*ones(length(time_store), 1), 'black')
plot(time_store,states(:,5) , 'r')
title('Vy');

subplot(2,3,6)
hold on
plot(time_store, x_des(6)*ones(length(time_store), 1), 'black')
plot(time_store,states(:,6) , 'r')
title('Vz');

velx = states(:,4);
vely = states(:,5);
velz = states(:,6);
vel = [velx vely velz];


figure()
subplot(2,2,1)
plot(time_store, states(:,7), 'r')
title('q1');

subplot(2,2,2)
plot(time_store, states(:,8), 'r')
title('q2');

subplot(2,2,3)
plot(time_store, states(:,9), 'r')
title('q3');

subplot(2,2,4)
plot(time_store, states(:,10), 'r')
title('q4');

quat1 = states(:,7);
quat2= states(:,8);
quat3 = states(:,9);
quat4 =  states(:,10);

quat = [quat1 quat2 quat3 quat4];





angvelx = states(:,11);
angvely = states(:,12);
angvelz = states(:,13);
omega  = [angvelx angvely angvelz];

figure()
subplot(2,2,1)
plot(time_store, states(:,11), 'r')
title('Omega_x');

subplot(2,2,2)
plot(time_store, states(:,12), 'r')
title('Omega_y');

subplot(2,2,3)
plot(time_store, states(:,13), 'r')
title('Omega_z');
 U = [control_ip];
 
figure()
 subplot(2,3,1)
 plot(time_store(1:end-1), control_ip(:,1), 'r')
 title('u1');
 
 subplot(2,3,2)
 plot(time_store(1:end-1), control_ip(:,2), 'r')
 title('u2');
 
 subplot(2,3,3)
 plot(time_store(1:end-1), control_ip(:,3), 'r')
 title('u3');
 
 subplot(2,3,4)
 plot(time_store(1:end-1), control_ip(:,4), 'r')
 title('u4');
 
 subplot(2,3,5)
 plot(time_store(1:end-1), control_ip(:,5), 'r')
 title('u5');
 
 subplot(2,3,6)
 plot(time_store(1:end-1), control_ip(:,6), 'r')
 title('u6');
 


