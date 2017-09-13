clear all
close all
load GNMSLog1.mat

%reformat
t = squeeze(t);
lv = squeeze(lv);

figure()
subplot(2,2,1)
plot(t, x(1,:), 'k', 'MarkerSize',1)
hold on;
%plot(t, xShot(1,:),  'b', 'MarkerSize',1)
legend('x*');
xlabel('t [sec]')
ylabel('x [m]')
title('position');

subplot(2,2,3)
plot(t(1:end-1), lv)
title('control update')
xlabel('t [sec]')
ylabel('F [N]')

subplot(2,2,4)
plot(t, lx(1,:)); hold on
title('state update')
xlabel('t [sec]')
ylabel('dx [m]')
legend('x update')


%%
figure();
plot(t(1:end-1), d(1,:)); hold on;
%plot(t(1:end-1), d(2,:))
title('defect')

