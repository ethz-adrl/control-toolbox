clear all
close all
load ../../../../../../../ct/GNMSLog2.mat

%reformat
t = squeeze(t);
lv = squeeze(lv);


plot(t, x(1,:))
hold on;
plot(t, xShot(1,:))
plot(t, x_rollout(1,:))
legend('x', 'xShot', 'xrollout');

%%
figure();
plot(t(1:end-1), lv)
title('gnms lv')

%%
figure();
plot(t(1:end-1), d(1,:)); hold on;
%plot(t(1:end-1), d(2,:))
title('defect')

%%
figure()
plot(t, lx(1,:))
title('gnms lx')
