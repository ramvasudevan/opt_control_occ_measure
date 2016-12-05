function PlotTraj(t_hist, x_hist, idx)

    figure;
    hold on;
    plot(x_hist(:,5), x_hist(:,7));

    if length(idx) == 1
        idx = round(linspace(1, length(t_hist), idx));
    end

    for i = 1 : length(idx)
        current_x = x_hist(idx(i),:);
        [ Mpos, Opos ] = FindPos(current_x);
        plot(Mpos(1), Mpos(2), 'ro', 'MarkerSize', 10);
        plot([Mpos(1),Opos(1)], [Mpos(2),Opos(2)], 'b', 'LineWidth',2);
    end
end

function [ Mpos, Opos ] = FindPos(x)
    params = SLIPParams;
    alpha = params.alpha;
    l0 = params.l0;
    if x(end) == 1      % Stance phase
        Mpos = [ x(5), x(1)*cos(x(3)) ];
        Opos = [ Mpos(1) + x(1)*sin(x(3)), 0 ];
    else                % Flight phase
        Mpos = [ x(5), x(7) ];
        Opos = Mpos;
%         Opos = [ nan, nan ];
%         Opos = Mpos + [ l0*sin(alpha), -l0*cos(alpha) ];
    end
end