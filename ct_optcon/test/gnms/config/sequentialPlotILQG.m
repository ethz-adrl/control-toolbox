clear all
close all
load iLQGLogInit.mat
load iLQGLog0.mat
load iLQGLog1.mat
load iLQGLog2.mat


%reformat
t = squeeze(t0);

figure()
subplot(2,3,1)
plot(t, squeeze(xInit), 'k--', 'MarkerSize',1); hold on;
plot(t, x0(1,:), 'k', 'MarkerSize',1); hold on;
plot(t, x1(1,:), 'b', 'MarkerSize',1); hold on;
plot(t, x2(1,:), 'r', 'MarkerSize',1); hold on;
legend('init', '1', '2', '3');
xlabel('t [sec]')
ylabel('x [m]')
title('position');

subplot(2,3,2)
plot(t(1:end-1), u_ffInit(1,:), 'k--', 'MarkerSize',1); hold on;
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
plot(1, cost0, 'o'); hold on;
plot(2, cost1, 'o'); hold on;
plot(3, cost2, 'o'); hold on;
title('cost')
xlabel('iteration')
ylabel('cost')




