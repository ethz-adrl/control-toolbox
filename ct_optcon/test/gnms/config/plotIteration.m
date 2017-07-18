function [] = plotIteration( fileName , iter, color)

load(fileName);


t = linspace(0, (K+2)*dt, K+1);
% load the current iteration

t=squeeze(t);

figure(1)
subplot(3,3,1)
plot(t, x(1,:), color,'MarkerSize',1);
%legend('init', '1', '2', '3');

subplot(3,3,2)
plot(t(1:end-1), u_ff(1,:), color, 'MarkerSize',1); hold on;
%    legend('init', '1', '2', '3');

subplot(3,3,3)
plot(iter, d_norm, strcat(color,'o'), 'MarkerSize', 8); hold on
% legend('1', '2', '3');

subplot(3,3,4)
%plot(t, squeeze(lx(1,:)), color); hold on
%legend('1', '2', '3');

subplot(3,3,5)
%plot(t(1:end-1), squeeze(luff), color); hold on;
%legend('1', '2', '3');

subplot(3,3,6)
plot(iter, cost, strcat(color,'o'), 'MarkerSize', 8);

subplot(3,3,7)
plot(iter, lx_norm, strcat(color,'o'), 'MarkerSize', 8);

subplot(3,3,8)
plot(iter, lu_norm, strcat(color,'o'), 'MarkerSize', 8);

end

