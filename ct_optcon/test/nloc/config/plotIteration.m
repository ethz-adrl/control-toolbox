function [] = plotIteration( fileName , iter, color, markerType, markerSize, jointNr, plotDefect)

% load the current iteration
load(fileName);

t = linspace(0, (K)*dt, K+1);

figure(1)
subplot(3,3,1)
plot(t, x(jointNr,:), color,'MarkerSize',1);
%legend('init', '1', '2', '3');


subplot(3,3,2)
plot(t(1:end-1), u_ff(jointNr,:), color, 'MarkerSize',1); hold on;
%    legend('init', '1', '2', '3');

subplot(3,3,3)
plot(iter, cost, strcat(color,markerType), 'MarkerSize', markerSize);

subplot(3,3,4)
plot(iter, lx_norm, strcat(color,markerType), 'MarkerSize', markerSize);

subplot(3,3,5)
plot(iter, lu_norm, strcat(color,markerType), 'MarkerSize', markerSize);

subplot(3,3,6)
plot(iter, alphaStep, strcat(color,markerType), 'MarkerSize', markerSize); hold on
%legend('1', '2', '3');

if plotDefect
    subplot(3,3,7, 'YScale', 'log')
    plot(iter, d_norm, strcat(color,markerType), 'MarkerSize', markerSize); hold on
    % legend('1', '2', '3');
    
    subplot(3,3,8)
    d_sq = squeeze(d);
    temp = size(d_sq);
    if(temp(2)>temp(1))
        normVec = sqrt(sum(d_sq.^2,1));
    else
        normVec = sqrt(sum(d_sq.^2,2));
    end
    plot(t, normVec, color);
    hold on;
    %legend('1', '2', '3');
    
end %plot defect

end

