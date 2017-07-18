clear all
close all


numIter = 14;    % number of iterations to be plotted

%% plot the init guess logs

plotInitGuesses('GNMSLogInit.mat')
plotInitGuesses('ILQRLogInit.mat')


%% plot the iterations
for iter= 0:1:numIter
    
    % plot the current iteration
    plotIteration(strcat(strcat('GNMSLog', num2str(iter)),'.mat'), iter, 'k');
    plotIteration(strcat(strcat('ILQRLog', num2str(iter)),'.mat'), iter, 'r');
    
end





