% Script for plotting EKF csv data
clear
close all

chaser_ekf = readmatrix('../iss_1_csv/441_test_2021-05-03-16-56-12_chaser/ekf.csv');
chaser_ekf_t = chaser_ekf(:,1)./1e9;
start_chaser_t = chaser_ekf_t(1);
chaser_ekf_t = chaser_ekf_t - start_chaser_t;

chaser_ekf_pos = chaser_ekf(:,6:8); % m
chaser_ekf_att = chaser_ekf(:,9:12); % quat
chaser_ekf_vel = chaser_ekf(:,13:15); % m/s
chaser_ekf_omega = chaser_ekf(:,16:18); % rad/s
chaser_ekf_acc = chaser_ekf(:,22:24); % m/s^2

figure(1);
plot(chaser_ekf_t(2:end), chaser_ekf_pos(2:end,1), 'r-', ...
     chaser_ekf_t(2:end), chaser_ekf_pos(2:end,2), 'g-', ...
     chaser_ekf_t(2:end), chaser_ekf_pos(2:end,3), 'b-');
title('Chaser EKF position');
xlabel('t (s)');
ylabel('(m)');
xlim([0, chaser_ekf_t(end)]);
grid on;

figure(2);
plot(chaser_ekf_t(2:end), chaser_ekf_vel(2:end,1), 'r-', ...
     chaser_ekf_t(2:end), chaser_ekf_vel(2:end,2), 'g-', ...
     chaser_ekf_t(2:end), chaser_ekf_vel(2:end,3), 'b-');
title('Chaser EKF velocity');
xlabel('t (s)');
ylabel('(m/s)');
xlim([0, chaser_ekf_t(end)]);
grid on;

figure(3);
plot(chaser_ekf_t(2:end), chaser_ekf_att(2:end,1), 'r-', ...
     chaser_ekf_t(2:end), chaser_ekf_att(2:end,2), 'g-', ...
     chaser_ekf_t(2:end), chaser_ekf_att(2:end,3), 'b-', ...
     chaser_ekf_t(2:end), chaser_ekf_att(2:end,4), 'k-');
title('Chaser EKF attitude');
xlabel('t (s)');
ylabel('(quat)');
xlim([0, chaser_ekf_t(end)]);
grid on;

figure(4);
plot(chaser_ekf_t(2:end), chaser_ekf_omega(2:end,1), 'r-', ...
     chaser_ekf_t(2:end), chaser_ekf_omega(2:end,2), 'g-', ...
     chaser_ekf_t(2:end), chaser_ekf_omega(2:end,3), 'b-');
title('Chaser EKF omega');
xlabel('t (s)');
ylabel('(rad/s)');
xlim([0, chaser_ekf_t(end)]);
grid on;

figure(5);
plot(chaser_ekf_t(2:end), chaser_ekf_acc(2:end,1), 'r-', ...
     chaser_ekf_t(2:end), chaser_ekf_acc(2:end,2), 'g-', ...
     chaser_ekf_t(2:end), chaser_ekf_acc(2:end,3), 'b-');
title('Chaser EKF acc');
xlabel('t (s)');
ylabel('m/s^2');
xlim([0, chaser_ekf_t(end)]);
grid on;
