% load('data/constV1_d4_T4');
% load('data/constV2_d4_T4');
% load('data/high_d4_T3');
load('data/high_d4_T5');

figure;
h_2d = gca;

hold on;
idx_mat = mode_hist(2:end) - mode_hist(1:end-1);
idx = [ find( idx_mat > 0 ); find( idx_mat < 0 )+1];
idx = [1; idx; length(mode_hist)-1];
springcoord( [ 0, 0 ] , [ 0, params.l0 ], 5, 0.3, 0.04);
plot( h_2d, x_hist, y_hist, '-.');
for i = 1 : length(idx)
    j = idx(i);
    if mode_hist(j+1) == mode_hist(j)
        xv = (x_hist(j+1) - x_hist(j)) / (t_hist(j+1) - t_hist(j));
        yv = (y_hist(j+1) - y_hist(j)) / (t_hist(j+1) - t_hist(j));
    elseif mode_hist(j) == mode_hist(j-1)
        xv = (x_hist(j) - x_hist(j-1)) / (t_hist(j) - t_hist(j-1));
        yv = (y_hist(j) - y_hist(j-1)) / (t_hist(j) - t_hist(j-1));
    end
    PlotFrame(state_hist(j,:),params,xv,yv,h_2d);
end
axis equal
ylim([0, 0.3]);
xlim([-1,0.6]);
xlabel('$x$','Interpreter','LaTex','FontSize',15);
ylabel('$y$','Interpreter','LaTex','FontSize',15);
box on;
set(h_2d, 'YTick', [0 1.5]);