clear all
close all
load ../../../../../../../ct/GNMSLog0.mat

%reformat
t = squeeze(t);
lv = squeeze(lv);

figure()
subplot(2,2,1)
plot(t, x(1,:), 'kx', 'MarkerSize',1)
hold on;
plot(t, xShot(1,:),  'bx', 'MarkerSize',1)
legend('x*', 'x init');
xlabel('t [sec]')
ylabel('x [m]')
title('positions');

subplot(2,2,2)
plot(t, x(2,:), 'k')
hold on;
plot(t, xShot(2,:),  'b')
legend('v*', 'v init');
xlabel('t [sec]')
ylabel('v [m/sec]')
title('velocities');


subplot(2,2,3)
plot(t(1:end-1), lv)
title('control update')
xlabel('t [sec]')
ylabel('F [N]')

subplot(2,2,4)
plot(t, lx(1,:)); hold on
plot(t, lx(2,:)); hold on
title('state update')
xlabel('t [sec]')
ylabel('dx [m]')
legend('x update', 'v update')


%%
figure();
plot(t(1:end-1), d(1,:)); hold on;
%plot(t(1:end-1), d(2,:))
title('defect')

