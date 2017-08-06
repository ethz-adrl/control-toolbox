clear all
close all


numIter = 9;    % number of iterations to be plotted

jointNr = 1;

%% plot the init guess logs

plotInitGuesses('GNMSLogInit.mat', ':k', 'x', 4, jointNr, true);
plotInitGuesses('ILQRLogInit.mat', ':r', 'o', 4, jointNr, false);


%% plot the iterations
for iter= 0:1:numIter
    
    % plot the current iteration
    plotIteration(strcat(strcat('GNMSLog', num2str(iter)),'.mat'), iter, 'k', 'x', 4, jointNr, true);
    plotIteration(strcat(strcat('ILQRLog', num2str(iter)),'.mat'), iter, '--r', 'o', 4,jointNr, false);
    
end





