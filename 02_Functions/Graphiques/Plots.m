function Plots(time, v, w)
% Compare les composantes X, Y, Z de deux tableaux en fonction du temps.
    X1 = v(1:3, :)';
    X2 = w(1:3, :)';
    
    figure;

    subplot(3,1,1);
    plot(time, X1(:,1), 'r', 'LineWidth', 1.5); hold on;
    plot(time, X2(:,1), 'b', 'LineWidth', 1.5);
    grid on; ylabel('X');
    title('Comparaison Estimate vs Corrected');
    legend('Estimate','Corrected');
    

    subplot(3,1,2);
    plot(time, X1(:,2), 'r', 'LineWidth', 1.5); hold on;
    plot(time, X2(:,2), 'b', 'LineWidth', 1.5);
    grid on; ylabel('Y');
    legend('Estimate','Corrected');
    
  
    subplot(3,1,3);
    plot(time, X1(:,3), 'r', 'LineWidth', 1.5); hold on;
    plot(time, X2(:,3), 'b', 'LineWidth', 1.5);
    grid on; ylabel('Z'); xlabel('Time');
    legend('Estimate','Corrected');

end