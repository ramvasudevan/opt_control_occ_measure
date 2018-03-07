%% Make movies
springcoord( [ 0, 0 ] , [ 0, params.l0 ], 5, 0.3, 0.04);
h_f = figure;
for i = 1 : length(t_hist)
    clf(h_f);
    hold on;
    plot(gca, x_hist, y_hist, '-.');
    PlotFrame(state_hist(i,:),params,[],[],gca);
    axis equal
    ylim([0, 0.35]);
    xlim([-1,0]);
    xlabel('$a$','Interpreter','LaTex','FontSize',15);
    ylabel('$b$','Interpreter','LaTex','FontSize',15);
    box on;
%     M(i) = getframe(h_f);
    if mod(i-1,20)==0
%         set(gcf, 'PaperPosition', [-1,-0.75,14,6]);
%         set(gcf, 'PaperSize', [12,2.5]);
        set(gca, 'XTick', [-1, 0]);
        set(gca, 'YTick', [0, 0.35]);
        box on;
%       saveas(gcf, ['movie/Fig', num2str((i-1)/20+1,'%03d')], 'pdf');
        print(gcf, '-dpdf', ['movie/Fig', num2str((i-1)/20+1,'%03d'), '.pdf']);
    end
end
close(h_f);
% out = M;
