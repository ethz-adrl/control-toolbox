function [] = plotSolverSummary(fileName, color, markerType, markerSize, plotDefect)

load(fileName);

figure(1)

subplot(2,3,1)
plot(iterations, totalCosts, strcat(color,markerType), 'MarkerSize', markerSize); 
hold on;
grid on;
title('cost');
xlabel('iteration')
ylabel('cost')
axis tight

%%
subplot(2,3,2, 'YScale', 'log');
plot(iterations, lx_norms, strcat(color,markerType), 'MarkerSize', markerSize); 
hold on; 
grid on;
title('lx norm')
xlabel('iteration')
axis tight

%%

subplot(2,3,3, 'YScale', 'log')
plot(iterations, lu_norms, strcat(color,markerType), 'MarkerSize', markerSize); 
hold on; grid on;
title('lu norm')
xlabel('iteration')
axis tight

%%

subplot(2,3,4)
plot(iterations, stepSizes, strcat(color,markerType), 'MarkerSize', markerSize); 
hold on
title('stepsize')
xlabel('iteration')
axis tight

%%
subplot(2,3,5)
if(plotDefect)
plot(iterations, defect_l1_norms, strcat(color,markerType), 'MarkerSize', markerSize);
hold on; grid on
title('total defect')
xlabel('iter')
ylabel('|d|')
axis tight
end

end

