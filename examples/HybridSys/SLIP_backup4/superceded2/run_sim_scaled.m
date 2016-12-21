% Simulation with scaled dynamics
% Vertical partition of flight phase (preferred)
% 
% 3 modes
% 

clear;
close all;

addpath('Utils');
addpath('PolyDynamics');
addpath('TrueDynamics');
addpath('Dynamics3');

params = SLIPParams;

scaling = 5;
previous_mode = 0;

current_mode = 3;
x0 = [ 0; 1.7; 1; 0 ];
controller = @(tt,xx) 0;

% current_mode = 1;
% x0 = [0.9;0;0;0;1];
% controller = @(tt,xx) 1;

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
    R_12 = @(xx) rescale_reset(@Reset_S2F_Approx0, xx, scale_x{1}, trans_x{1}, scale_x{2}, trans_x{2});
    R_23 = @(xx) rescale_reset(@(var) var, xx, scale_x{2}, trans_x{2}, scale_x{3}, trans_x{3});
    R_31 = @(xx) rescale_reset(@Reset_F2S_Approx, xx, scale_x{3}, trans_x{3}, scale_x{1}, trans_x{1});
end

current_time = 0;

P = SLIPPlot( current_mode, x0, params );
t_hist = [];
x_hist = [];
x_dot_hist = [];
y_hist = [];
y_dot_hist = [];
l_hist = [];
l_dot_hist = [];
theta_hist = [];
theta_dot_hist = [];

x0 = rescale_state( x0, domain_size{current_mode} );

while current_time < MaxTime - 0.05
    disp(current_mode);
    switch current_mode
        case 1      % Stance
            options = odeset('Events',@EvtFunc_S2F_scaled);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
                [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{1}(xx) + g{1}(xx) * controller(tt,xx) ), ...
                             (current_time : 0.01 : 1), x0, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                xend = xout(end,:);
                x0 = R_12(xend);
                current_mode = 2;
            end
            current_time = tout(end);
            % Plot
            tout = tout * scaling;
            xout = rescale_state_back( xout, domain_size{1} );
            t_hist = [t_hist; tout];
            x_hist = [x_hist; xout(:,5)];
            x_dot_hist = [x_dot_hist; NaN * xout(:,1)];
            y_hist = [y_hist; NaN * xout(:,1)];
            y_dot_hist = [y_dot_hist; NaN * xout(:,1)];
            l_hist = [l_hist; xout(:,1)];
            l_dot_hist = [l_dot_hist; xout(:,2)];
            theta_hist = [theta_hist; xout(:,3)];
            theta_dot_hist = [theta_dot_hist; xout(:,4)];
            
            P.Visualize( tout, xout, previous_mode );
        case 2      % Flight 1 (under ground)
            options = odeset('Events',@EvtFunc_F1_scaled);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{2}(xx) ), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                xend = xout(end,:);
                x0 = R_23(xend);
                current_mode = 3;
            end
            current_time = tout(end);
            % Plot
            tout = tout * scaling;
            xout = rescale_state_back( xout, domain_size{2} );
            t_hist = [t_hist; tout];
            x_hist = [x_hist; xout(:,1)];
            x_dot_hist = [x_dot_hist; xout(:,2)];
            y_hist = [y_hist; xout(:,3)];
            y_dot_hist = [y_dot_hist; xout(:,4)];
            l_hist = [l_hist; NaN * xout(:,1)];
            l_dot_hist = [l_dot_hist; NaN * xout(:,1)];
            theta_hist = [theta_hist; NaN * xout(:,1)];
            theta_dot_hist = [theta_dot_hist; NaN * xout(:,1)];
            
            P.Visualize( tout, xout, previous_mode );
            
        case 3      % Flight 2 (above ground)
            options = odeset('Events',@EvtFunc_F2S_scaled);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{3}(xx) ), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                xend = xout(end,:);
                x0 = R_31(xend);
                current_mode = 1;
            end
            current_time = tout(end);
            % Plot
            tout = tout * scaling;
            xout = rescale_state_back( xout, domain_size{3} );
            t_hist = [t_hist; tout];
            x_hist = [x_hist; xout(:,1)];
            x_dot_hist = [x_dot_hist; xout(:,2)];
            y_hist = [y_hist; xout(:,3)];
            y_dot_hist = [y_dot_hist; xout(:,4)];
            l_hist = [l_hist; NaN * xout(:,1)];
            l_dot_hist = [l_dot_hist; NaN * xout(:,1)];
            theta_hist = [theta_hist; NaN * xout(:,1)];
            theta_dot_hist = [theta_dot_hist; NaN * xout(:,1)];
            
            P.Visualize( tout, xout, previous_mode );
            
        otherwise
            error('Invalid Mode');
    end
end

% figure(2);
% hold on;
% plot(t_hist, x_dot_hist,'LineWidth',2);
% title('x dot');
% 
% figure(2);
% hold on;
% plot(t_hist, y_hist, 'LineWidth', 2);
% title('y');
% 
% figure;
% plot(t_hist, y_dot_hist, 'LineWidth',2);
% title('y dot');
% 
% figure;
% plot(t_hist, theta_hist, 'LineWidth', 2);
% title('theta');
% 
% figure;
% plot(t_hist, theta_dot_hist, 'LineWidth', 2);
% title('theta dot');
% 
figure;
plot(t_hist, l_hist, 'LineWidth', 2);
title('l');
% 
% figure;
% plot(t_hist, l_dot_hist, 'LineWidth', 2);
% title('l dot');

