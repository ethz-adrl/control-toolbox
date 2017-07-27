function [] = plotIteration( fileName , iter, color, markerType, jointNr, plotDefect)

load(fileName);


t = linspace(0, (K+2)*dt, K+1);
% load the current iteration

t=squeeze(t);

figure(1)
subplot(3,3,1)
plot(t, x(jointNr,:), color,'MarkerSize',1);
%legend('init', '1', '2', '3');

subplot(3,3,2)
plot(t(1:end-1), u_ff(jointNr,:), color, 'MarkerSize',1); hold on;
%    legend('init', '1', '2', '3');

if plotDefect
subplot(3,3,3, 'YScale', 'log')
plot(iter, d_norm, strcat(color,markerType), 'MarkerSize', 8); hold on
% legend('1', '2', '3');
end

subplot(3,3,4)
plot(iter, alphaStep, strcat(color,markerType), 'MarkerSize', 8); hold on
%legend('1', '2', '3');

subplot(3,3,5)
%plot(t(1:end-1), squeeze(luff), color); hold on;
%legend('1', '2', '3');

subplot(3,3,6)
plot(iter, cost, strcat(color,markerType), 'MarkerSize', 8);

subplot(3,3,7)
plot(iter, lx_norm, strcat(color,markerType), 'MarkerSize', 8);

subplot(3,3,8)
plot(iter, lu_norm, strcat(color,markerType), 'MarkerSize', 8);

end

