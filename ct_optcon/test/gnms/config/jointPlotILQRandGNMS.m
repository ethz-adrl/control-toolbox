clear all
close all


numIter = 6;    % number of iterations to be plotted

jointNr = 1;

%% plot the init guess logs

plotInitGuesses('GNMSLogInit.mat', '--k', 'x', jointNr);
plotInitGuesses('ILQRLogInit.mat', '--r', 'o', jointNr);


%% plot the iterations
for iter= 0:1:numIter
    
    % plot the current iteration
    plotIteration(strcat(strcat('GNMSLog', num2str(iter)),'.mat'), iter, 'k', 'x', jointNr);
    plotIteration(strcat(strcat('ILQRLog', num2str(iter)),'.mat'), iter, 'r', 'o',jointNr);
    
end





