close all
clear all
load 'solution.mat'


% number of variables in workspace
numVars = length(who)/4;

figure(1); hold on;
figure(2); hold on;

mult = 1;

for i=0:mult:numVars*mult-1
    
    x_var = eval(genvarname(strcat('x_',num2str(i))));
    t_var = squeeze(eval(genvarname(strcat('t_',num2str(i)))));
    ts_var = eval(genvarname(strcat('ts_',num2str(i))));

figure(1);
plot([ts_var ts_var],[-5 20], ':k');
plot(ts_var+t_var, x_var(1,:));
grid on

figure(2);
plot([ts_var ts_var],[-5 20], ':k');
plot(ts_var+t_var, x_var(2,:));
grid on
end
