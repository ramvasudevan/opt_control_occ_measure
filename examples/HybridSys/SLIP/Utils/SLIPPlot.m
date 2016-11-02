classdef SLIPPlot < handle
    
    properties (SetAccess = public)
        state_hist;
        time_hist;
        current_q;      % [ x, y, l, theta ]
        phase;
        Mpos;
        Opos;
        params;
    end
    
    properties (SetAccess = private)
        h_2d;
        h_M;
        h_O;
        h_link;
        h_gnd
    end
    
    methods
        % Constructor
        function P = SLIPPlot(phase, x0, params)
            P.phase = phase;
            P.params = params;
            switch P.phase
                case 1  % Stance
                    ll = x0(1);
                    tt = x0(3);
                    P.current_q(1) = x0(5) - ll * sin(tt);
                    P.current_q(2) = ll * cos(tt);
                    P.current_q(3) = ll;
                    P.current_q(4) = tt;
                case {2,3} % Flight 1 (under ground), or Flight 2 (above ground)
                    P.current_q(1) = x0(1);
                    P.current_q(2) = x0(3);
                    P.current_q(3) = params.l0;
                    P.current_q(4) = params.alpha;
                otherwise
                    error('Phase index error.');
            end
            
            figure;
            xlim([-1,10]);
            ylim([-0.5,3]);
            P.h_2d = gca;
            hold( P.h_2d, 'on' );
            P.UpdatePos;
            P.h_M = plot(P.h_2d, ...
                         P.Mpos(1), P.Mpos(2), 'ro', 'MarkerSize', 10);
%             P.h_O = plot(P.h_2d, ...
%                          P.Opos(1), P.Opos(2), 'ro', 'MarkerSize', 10);
            P.h_link = plot(P.h_2d, ...
                            [P.Mpos(1), P.Opos(1)], ...
                            [P.Mpos(2), P.Opos(2)], ...
                            '-b', 'LineWidth', 4 );
            P.h_gnd = plot(P.h_2d, ...
                           [-1, 11], [0,0], '-k', 'LineWidth', 2);
            hold( P.h_2d, 'off' );
            title( P.h_2d, 0 );
        end
        
        function UpdatePlot( P, tval, xval )
            switch P.phase
                case 1  % Stance
                    ll = xval(1);
                    tt = xval(3);
                    P.current_q(1) = xval(5) - ll * sin(tt);
                    P.current_q(2) = ll * cos(tt);
                    P.current_q(3) = ll;
                    P.current_q(4) = tt;
                case {2,3} % Flight 1 (under ground), or Flight 2 (above ground)
                    P.current_q(1) = xval(1);
                    P.current_q(2) = xval(3);
                    P.current_q(3) = P.params.l0;
                    P.current_q(4) = P.params.alpha;
                otherwise
                    error('Phase index error.');
            end
            P.UpdatePos;
            set( P.h_M, ...
                 'XData', P.Mpos(1), ...
                 'YData', P.Mpos(2) );
            set( P.h_O, ...
                 'XData', P.Opos(1), ...
                 'YData', P.Opos(2) );
            set ( P.h_link, ...
                  'XData', [P.Mpos(1), P.Opos(1)], ...
                  'YData', [P.Mpos(2), P.Opos(2)] );
            title( P.h_2d, tval );
            
            drawnow;
        end
            
        function UpdatePos( P )
            ll = P.current_q(3);
            tt = P.current_q(4);
            P.Mpos = P.current_q(1:2);
            P.Opos = P.Mpos + [ll*sin(tt), -ll*cos(tt)];
        end
        
        function Visualize( P, t_hist, x_hist, phase )
            P.phase = phase;
            for i = 2 : length(t_hist)
                tic;
                tval = t_hist(i);
                xval = x_hist(i,:);
                P.UpdatePlot( tval, xval );
                tstep = 1 * (t_hist(i) - t_hist(i-1));
                t = toc;
                tstep = tstep * 0.1;
                if (t < tstep)
                    pause(tstep - t);
                end
            end
        end
    end
end
            
            