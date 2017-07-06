clear all
close all
load GNMSLogInit.mat
load GNMSLog0.mat
load GNMSLog1.mat
load GNMSLog2.mat


%reformat
t = squeeze(t);
lv = squeeze(lv);

figure()
subplot(2,3,1)
plot(t, xinit(1,:), 'k--', 'MarkerSize',1); hold on;
plot(t, x0(1,:), 'k', 'MarkerSize',1); hold on;
plot(t, x1(1,:), 'b', 'MarkerSize',1); hold on;
plot(t, x2(1,:), 'r', 'MarkerSize',1); hold on;
legend('init', '1', '2', '3');
xlabel('t [sec]')
ylabel('x [m]')
title('position');

subplot(2,3,2)
plot(t(1:end-1), uinit(1,:), 'k--', 'MarkerSize',1); hold on;
plot(t(1:end-1), u0(1,:), 'k', 'MarkerSize',1); hold on;
plot(t(1:end-1), u1(1,:), 'b', 'MarkerSize',1); hold on;
plot(t(1:end-1), u2(1,:), 'r', 'MarkerSize',1); hold on;
legend('init', '1', '2', '3');
xlabel('t [sec]')
ylabel('x [m]')
title('control');

subplot(2,3,3)
plot(t(1:end-1), squeeze(lv0)); hold on;
plot(t(1:end-1), squeeze(lv1)); hold on;
plot(t(1:end-1), squeeze(lv2)); hold on;
legend('1', '2', '3');
title('control update')
xlabel('t [sec]')
ylabel('F [N]')

subplot(2,3,4)
plot(t, squeeze(lx0(1,:))); hold on
plot(t, squeeze(lx1(1,:))); hold on
plot(t, squeeze(lx2(1,:))); hold on
title('state update')
xlabel('t [sec]')
ylabel('dx [m]')
legend('1', '2', '3');


subplot(2,3,5)
plot(t(1:end-1), squeeze(d0(1,:))); hold on
plot(t(1:end-1), squeeze(d1(1,:))); hold on
plot(t(1:end-1), squeeze(d2(1,:))); hold on
title('defect')
xlabel('t [sec]')
ylabel('d [m]')
legend('1', '2', '3');





