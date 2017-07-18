function [] = plotInitGuesses( fileName )

load(fileName);

t = linspace(0, (K+2)*dt, K+1);

figure(1)
subplot(3,3,1)
plot(t, x(1,:), 'k', 'MarkerSize',1); hold on;
xlabel('t [sec]')
ylabel('x [m]')
title('position');

subplot(3,3,2)
plot(t(1:end-1), u_ff(1,:), 'k', 'MarkerSize',1); hold on;
xlabel('t [sec]')
title('control');

subplot(3,3,3)
plot(-1, d_norm, 'o', 'MarkerSize', 8); hold on
title('defect norm')
xlabel('t [sec]')
ylabel('d [m]')

subplot(3,3,4)
hold on
title('state update')
xlabel('t [sec]')
ylabel('dx [m]')

subplot(3,3,5)
hold on;
title('control update')
xlabel('t [sec]')
ylabel('F [N]')

subplot(3,3,6)
plot(-1, cost, 'o'); hold on;
title('cost')
xlabel('iteration')
ylabel('cost')

subplot(3,3,7, 'YScale', 'log');
hold on; grid on;
title('lx norm')
xlabel('iteration')

subplot(3,3,8, 'YScale', 'log')
hold on; grid on;
title('lu norm')
xlabel('iteration')

end

