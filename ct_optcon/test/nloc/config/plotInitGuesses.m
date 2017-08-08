function [] = plotInitGuesses(fileName, color, markerType, markerSize, jointNr, plotDefect)

load(fileName);

t = linspace(0, (K)*dt, K+1);

figure(1)
subplot(3,3,1)
plot(t, x(jointNr,:), color, 'MarkerSize',markerSize); hold on;
xlabel('t [sec]')
ylabel('x')
title('position (1-dim)');
axis tight

subplot(3,3,2)
plot(t(1:end-1), u_ff(jointNr,:), color, 'MarkerSize',markerSize); hold on;
xlabel('t [sec]')
ylabel('u')
title('control input (1-dim)');
axis tight

subplot(3,3,3)
plot(-1, cost, strcat(color,markerType), 'MarkerSize', markerSize); 
hold on;
grid on;
title('cost');
xlabel('iteration')
ylabel('cost')
axis tight

subplot(3,3,4, 'YScale', 'log');
hold on; 
grid on;
title('lx norm')
xlabel('iteration')
axis tight

subplot(3,3,5, 'YScale', 'log')
hold on; grid on;
title('lu norm')
xlabel('iteration')
axis tight

subplot(3,3,6)
hold on
title('stepsize')
xlabel('iteration')
axis tight

if plotDefect
subplot(3,3,7)
plot(-1, d_norm, strcat(color,markerType), 'MarkerSize', markerSize);
hold on; grid on
title('total defect')
xlabel('iter')
ylabel('|d|')
axis tight
end
 
subplot(3,3,8)
title('defect norms')
xlabel('t [sec]')
ylabel('|d(t)|')
d_sq = squeeze(d);
normVec = sqrt(sum(d_sq.^2,1));
plot(t, normVec, color);
axis tight
hold on


end

