clear all
close all


numIter = 0;    % number of iterations to be plotted

jointNr = 1;

%% plot the init guess logs

plotInitGuesses('GNMSLogInit.mat', '--k', 'x', jointNr, true);
plotInitGuesses('ILQRLogInit.mat', '--r', 'o', jointNr, false);


%% plot the iterations
for iter= 0:1:numIter
    
    % plot the current iteration
    plotIteration(strcat(strcat('GNMSLog', num2str(iter)),'.mat'), iter, 'k', 'x', jointNr, true);
    plotIteration(strcat(strcat('ILQRLog', num2str(iter)),'.mat'), iter, 'r', 'o',jointNr, false);
    
end





