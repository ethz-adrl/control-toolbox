clear all
close all


numIter = 5;    % number of iterations to be plotted

%% plot the init guess logs
load GNMSLogInit.mat

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

subplot(3,3,7, 'XScale', 'log', 'YScale', 'log');
hold on; grid on;
title('lx norm')
xlabel('iteration')

subplot(3,3,8, 'XScale', 'log', 'YScale', 'log')
hold on; grid on;
title('lu norm')
xlabel('iteration')

%% plot the iterations
for iter= 0:1:numIter
    
    % load the current iteration
    load(strcat(strcat('GNMSLog', num2str(iter)),'.mat'));
    
    t=squeeze(t);
        
    figure(1)
    subplot(3,3,1)
    plot(t, x(1,:), 'MarkerSize',1); hold on;
    %legend('init', '1', '2', '3');
    
    subplot(3,3,2)
    plot(t(1:end-1), u_ff(1,:), 'MarkerSize',1); hold on;
    %    legend('init', '1', '2', '3');
    
    subplot(3,3,3)
    plot(iter, d_norm, 'o', 'MarkerSize', 8); hold on
    % legend('1', '2', '3');
    
    subplot(3,3,4)
    plot(t, squeeze(lx(1,:))); hold on
    %legend('1', '2', '3');
    
    subplot(3,3,5)
    plot(t(1:end-1), squeeze(luff)); hold on;
    %legend('1', '2', '3');
    
    subplot(3,3,6)
    plot(iter, cost, 'o', 'MarkerSize', 8); hold on;
    
    subplot(3,3,7, 'XScale', 'log', 'YScale', 'log')
    plot(iter, lx_norm, 'o', 'MarkerSize', 8); 
    
    subplot(3,3,8)
    plot(iter, lu_norm, 'o', 'MarkerSize', 8);
end





