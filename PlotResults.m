function PlotResults(speed, obstacle)

    color_share = [0 0.4470 0.7410];
    color_nominal = [0.4660 0.6740 0.1880];
    color_contingency = [0.9290 0.6940 0.1250];
    color_obstacle = [0.6350 0.0780 0.1840];    
    
    subplot(2,2,1);
    plot(speed.share.t, speed.share.s, '-', 'LineWidth', 1.5, 'Color', color_share);
    hold on;
    plot(speed.nominal.t, speed.nominal.s, '-', 'LineWidth', 1.5, 'Color', color_nominal);
    hold on;
    plot(speed.contingency.t, speed.contingency.s, '-', 'LineWidth', 1.5, 'Color', color_contingency);
    hold on;
    plot(obstacle.t, obstacle.s, '-', 'LineWidth', 1.5, 'Color', color_obstacle);
    
    title('s - t');
    xlabel("t");
    xlabel("s");
    legend("Share", "Nominal", "Contingency", "Obstacle");
    grid on;
    
    subplot(2,2,2);
    plot(speed.share.t, speed.share.v, '-', 'LineWidth', 1.5, 'Color', color_share);
    hold on;
    plot(speed.nominal.t, speed.nominal.v, '-', 'LineWidth', 1.5, 'Color', color_nominal);
    hold on;
    plot(speed.contingency.t, speed.contingency.v, '-', 'LineWidth', 1.5, 'Color', color_contingency);
    hold on;
    plot(obstacle.t, obstacle.v, '-', 'LineWidth', 1.5, 'Color', color_obstacle);
    
    title('v - t');
    xlabel("t");
    xlabel("s");
    legend("Share", "Nominal", "Contingency", "Obstacle");
    grid on;
    
    subplot(2,2,3);
    plot(speed.share.t, speed.share.a, '-', 'LineWidth', 1.5, 'Color', color_share);
    hold on;
    plot(speed.nominal.t, speed.nominal.a, '-', 'LineWidth', 1.5, 'Color', color_nominal);
    hold on;
    plot(speed.contingency.t, speed.contingency.a, '-', 'LineWidth', 1.5, 'Color', color_contingency);
    
    title('a - t');
    xlabel("t");
    xlabel("s");
    legend("Share", "Nominal", "Contingency");
    grid on;
    
    subplot(2,2,4);
    plot(speed.share.t, speed.share.j, '-', 'LineWidth', 1.5, 'Color', color_share);
    hold on;
    plot(speed.nominal.t, speed.nominal.j, '-', 'LineWidth', 1.5, 'Color', color_nominal);
    hold on;
    plot(speed.contingency.t, speed.contingency.j, '-', 'LineWidth', 1.5, 'Color', color_contingency);
    
    title('j - t');
    xlabel("t");
    xlabel("s");
    legend("Share", "Nominal", "Contingency");
    grid on;
    
end