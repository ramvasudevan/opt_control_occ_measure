function [out] = plotdata( filename, h_2d, dist )

if isempty(filename)
    load('data/Far_T6_d6_dist4');
    out.u{1} = out.u{1}*0 - 1;
else
    load(filename);
    disp(['Processing ', filename]);
end

current_mode = 3;
x0 = [ 0; 1.7; 1; 0 ];

% params = SLIPParams;

opt = [ ...     % 1 = actual, 2 = taylor expansion
        1;      % Dynamics
        1;      % Guard
        1;      % Reset map
       ];

MaxTime = 1;

domain_size = params.domain_size;
scale_x = cell(3,1);
trans_x = cell(3,1);
f = cell(3,1);
g = cell(3,1);
for i = 1 : 3
    scale_x{i} = (domain_size{i}(:,2) - domain_size{i}(:,1)) / 2;
    trans_x{i} = mean(domain_size{i},2);
end
% Dynamics
if opt(1)
    f{1} = @(xx) scaling * rescale_dynamics(@(x) Stance_f(x), xx, scale_x{1}, trans_x{1}, scale_x{1});
    g{1} = @(xx) scaling * rescale_dynamics(@(x) Stance_g(x), xx, scale_x{1}, trans_x{1}, scale_x{1});
else
    f{1} = @(xx) scaling * rescale_dynamics(@(x) Stance_f_Approx(x), xx, scale_x{1}, trans_x{1}, scale_x{1});
    g{1} = @(xx) scaling * rescale_dynamics(@(x) Stance_g_Approx(x), xx, scale_x{1}, trans_x{1}, scale_x{1});
end
f{2} = @(xx) scaling * rescale_dynamics(@(x) Flight_f(x), xx, scale_x{2}, trans_x{2}, scale_x{2});
g{2} = @(xx) scaling * rescale_dynamics(@(x) Flight_g(x), xx, scale_x{2}, trans_x{2}, scale_x{2});
f{3} = @(xx) scaling * rescale_dynamics(@(x) Flight_f(x), xx, scale_x{3}, trans_x{3}, scale_x{3});
g{3} = @(xx) scaling * rescale_dynamics(@(x) Flight_g(x), xx, scale_x{3}, trans_x{3}, scale_x{3});
% Reset map
if opt(3)
    R_12 = @(xx) rescale_reset(@Reset_S2F, xx, scale_x{1}, trans_x{1}, scale_x{2}, trans_x{2});
    R_23 = @(xx) rescale_reset(@(var) var, xx, scale_x{2}, trans_x{2}, scale_x{3}, trans_x{3});
    R_31 = @(xx) rescale_reset(@Reset_F2S, xx, scale_x{3}, trans_x{3}, scale_x{1}, trans_x{1});
else
    R_12 = @(xx) rescale_reset(@Reset_S2F_Approx, xx, scale_x{1}, trans_x{1}, scale_x{2}, trans_x{2});
    R_23 = @(xx) rescale_reset(@(var) var, xx, scale_x{2}, trans_x{2}, scale_x{3}, trans_x{3});
    R_31 = @(xx) rescale_reset(@Reset_F2S_Approx, xx, scale_x{3}, trans_x{3}, scale_x{1}, trans_x{1});
end

controller = @(tt,xx) double(subs(out.u{1}, [t;x{1}], [tt;xx]));

current_time = 0;

state_hist = [];        % [ l, ldot, theta, thetadot, x, xdot, y, ydot, mode ]
t_hist = [];

x0 = rescale_state( x0, domain_size{current_mode} );

while current_time < MaxTime - 0.05
    disp(current_mode);
    switch current_mode
        case 1      % Stance
            options = odeset('Events',@EvtFunc_S2F_scaled);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
                [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{1}(xx) + g{1}(xx) * controller(tt,xx) ), ...
                             (current_time : 0.005 : 1), x0, options);
            % Reset
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    x0 = R_12(xend);
                    current_mode = 2;
                end
            end
            current_time = tout(end);
            % Plot
            tout = tout * scaling;
            xout = rescale_state_back( xout, domain_size{1} );
            t_hist = [ t_hist; tout ];
            mat = [ eye(5), nan*ones(5,3) ];
            tmp = xout*mat;
            tmp(:,7) = tmp(:,1) .* cos(tmp(:,3));
            state_hist = [ state_hist; tmp, 1*ones(length(tout),1) ];
            
        case 2      % Flight 1 (under ground)
            options = odeset('Events',@EvtFunc_F1_scaled);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{2}(xx) ), ...
                             (current_time : 0.005 : MaxTime), x0, options);
            % Reset
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    x0 = R_23(xend);
                    current_mode = 3;
                end
            end
            current_time = tout(end);
            % Plot
            tout = tout * scaling;
            xout = rescale_state_back( xout, domain_size{2} );
            t_hist = [t_hist; tout];
            mat = [ nan*ones(4,4), eye(4) ];
            state_hist = [ state_hist; xout*mat, 2*ones(length(tout),1) ];
            
        case 3      % Flight 2 (above ground)
            options = odeset('Events',@EvtFunc_F2S_scaled);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{3}(xx) ), ...
                             (current_time : 0.005 : MaxTime), x0, options);
            % Reset
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    x0 = R_31(xend);
                    current_mode = 1;
                end
            end
            current_time = tout(end);
            % Plot
            tout = tout * scaling;
            xout = rescale_state_back( xout, domain_size{3} );
            t_hist = [t_hist; tout];
            mat = [ nan*ones(4,4), eye(4) ];
            state_hist = [ state_hist; xout*mat, 3*ones(length(tout),1) ];
            
        case 0
            break;
            
        otherwise
            error('Invalid Mode');
    end
end

t_hist = t_hist( state_hist(:,5)<=dist+0.5 );
state_hist = state_hist( state_hist(:,5)<=dist+0.5, : );
l_hist          = state_hist(:,1);
l_dot_hist      = state_hist(:,2);
theta_hist      = state_hist(:,3);
theta_dot_hist  = state_hist(:,4);
x_hist          = state_hist(:,5);
x_dot_hist      = state_hist(:,6);
y_hist          = state_hist(:,7);
y_dot_hist      = state_hist(:,8);
mode_hist       = state_hist(:,9);


% %% Plot the control
% u_hist = 0 * t_hist;
% for i = 1 : length(t_hist)
%     if ~isnan(l_hist(i))
%         s = rescale_state( [ l_hist(i); l_dot_hist(i); theta_hist(i); theta_dot_hist(i); x_hist(i) ], ...
%                            domain_size{1} );
%         u_hist(i) = controller( t_hist(i)/scaling, s );
%     else
%         u_hist(i) = nan;
%     end
% end
% u_hist = (u_hist + 1) / 10;
% 
% figure;
% plot(t_hist, u_hist);

%% Key frames
hold on;
idx_mat = mode_hist(2:end) - mode_hist(1:end-1);
idx = [ find( idx_mat > 0 ); find( idx_mat < 0 )+1];
idx = [1; idx];
springcoord( [ 0, 0 ] , [ 0, params.l0 ], 5, 1.5, 0.2);
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
ylim([0, 1.5]);
xlim([0,10]);
set(h_2d, 'YTick', [0 1.5]);

%% Make movies
h_f = figure;
for i = 1 : length(t_hist)
    clf(h_f);
    hold on;
    plot(gca, x_hist, y_hist, '-.');
    PlotFrame(state_hist(i,:),params,[],[],gca);
    axis equal
    ylim([0,1.5]);
    xlim([0,10]);
    set(gca, 'YTick', [0 1.5]);
    set(gca, 'XTick', 0:2:10);
    xlabel('x');
    ylabel('y');
    box on;
%     M(i) = getframe(h_f);
%     if mod(i,20)==0
      set(gcf, 'PaperPosition', [-1.2,-0.75,14,4]);
      set(gcf, 'PaperSize', [12,2.5]);
      saveas(gcf, ['Fig', num2str(i,'%03d')], 'pdf');
%     end
end
close(h_f);
% out = M;

