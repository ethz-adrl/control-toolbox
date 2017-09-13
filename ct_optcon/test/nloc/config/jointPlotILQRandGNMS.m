clear all
close all


numIter = 19;    % number of iterations to be plotted

jointNr = 1;

markerSize = 7;

%% plot the init guess logs

plotInitGuesses('GNMSLogInit.mat', ':k', 'x', markerSize, jointNr, true);
plotInitGuesses('ILQRLogInit.mat', ':r', 'o', markerSize, jointNr, true);


%% plot the iterations
for iter= 0:1:numIter
    
    % plot the current iteration
    plotIteration(strcat(strcat('ILQRLog', num2str(iter)),'.mat'), iter, 'k', 'x', markerSize, jointNr, true);
    plotIteration(strcat(strcat('ILQR_GNMS_5Log', num2str(iter)),'.mat'), iter, '--r', 'o', markerSize,jointNr, true);
    
end

%%
subplot(3,3,9)
hold on;
plot(-1, 0.1, 'ro', 'MarkerSize', markerSize); % move x-coordinate out of axis limits
plot(-1,-0.1, 'kx', 'MarkerSize', markerSize); % move x-coordinate out of axis limits
ylim([-1 1])
xlim([-0.1, 1])
set(gca,'XTick',[])
set(gca,'YTick',[])
set(gca,'XTickLabel','')
set(gca,'YTickLabel','')
title('Legend')
legend('GNMS', 'ILQR')
legend boxoff



