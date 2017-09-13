clear all
close all

color = 'b';
markerType = 'o';
markerSize = 5;

plotSolverSummary('ilqrSummary.mat', ':k', 'x', markerSize, true);
plotSolverSummary('gnmsSummary.mat', 'r', 'o', markerSize, true);




