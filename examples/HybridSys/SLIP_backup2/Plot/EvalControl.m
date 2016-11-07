% Plug the control input into the (scaled) system
% 

% clear;
close all;

addpath('Utils');
addpath('PolyDynamics');
addpath('TrueDynamics');
addpath('Dynamics3');

params = SLIPParams;
controller = @(tt,xx) double(subs(out.u{1,1}, [t;x{1}], [tt;xx]));
% current_mode = 3;       % 1 = Stance; 2 = Flight, under; 3 = Flight, above
% x0 = [ 0; 1.5; 1.2; 0 ];

current_mode = 3;
x0 = [ 0; 1.7; 1; 0 ];

opt = [ ...     % 1 = actual, 2 = taylor expansion
        1;      % Dynamics
        1;      % Guard
        1;      % Reset map
       ];

MaxTime = 6;
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

while current_time < MaxTime - 0.1
    disp(current_mode);
    switch current_mode
        case 1      % Stance
            options = odeset('Events',@EvtFunc_S2F);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            if opt(1)
                [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Stance_f(x) + ...
                             Stance_g(x) * controller(t,x), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            else
                [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Stance_f_Approx(x) + ...
                             Stance_g_Approx(x) * controller(t,x), ...
                             (current_time : 0.02 : MaxTime), x0, options);
            end
            current_time = tout(end);
            t_hist = [t_hist; tout];
            x_hist = [x_hist; xout(:,5)];
            x_dot_hist = [x_dot_hist; NaN * xout(:,1)];
            y_hist = [y_hist; NaN * xout(:,1)];
            y_dot_hist = [y_dot_hist; NaN * xout(:,1)];
            l_hist = [l_hist; xout(:,1)];
            l_dot_hist = [l_dot_hist; xout(:,2)];
            theta_hist = [theta_hist; xout(:,3)];
            theta_dot_hist = [theta_dot_hist; xout(:,4)];
            
            P.Visualize( tout, xout, current_mode );
            % Reset
            if ~isempty(event_id)
                xend = xout(end,:);
                x0 = Reset_S2F(xend,params);
                if (xend(3)+params.alpha<0)
                    current_mode = 2;
                else
                    current_mode = 3;
                end
            end
        case 2      % Flight 1 (under ground)
            options = odeset('Events',@EvtFunc_F1);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Flight_f(x,params), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            current_time = tout(end);
            t_hist = [t_hist; tout];
            x_hist = [x_hist; xout(:,1)];
            x_dot_hist = [x_dot_hist; xout(:,2)];
            y_hist = [y_hist; xout(:,3)];
            y_dot_hist = [y_dot_hist; xout(:,4)];
            l_hist = [l_hist; NaN * xout(:,1)];
            l_dot_hist = [l_dot_hist; NaN * xout(:,1)];
            theta_hist = [theta_hist; NaN * xout(:,1)];
            theta_dot_hist = [theta_dot_hist; NaN * xout(:,1)];
            
            
            P.Visualize( tout, xout, current_mode );
            % Reset
            if ~isempty(event_id)
                xend = xout(end,:);
                x0 = xend;
                current_mode = 3;
            end
        case 3      % Flight 2 (above ground)
            options = odeset('Events',@EvtFunc_F2S);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Flight_f(x,params), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            current_time = tout(end);
            t_hist = [t_hist; tout];
            x_hist = [x_hist; xout(:,1)];
            x_dot_hist = [x_dot_hist; xout(:,2)];
            y_hist = [y_hist; xout(:,3)];
            y_dot_hist = [y_dot_hist; xout(:,4)];
            l_hist = [l_hist; NaN * xout(:,1)];
            l_dot_hist = [l_dot_hist; NaN * xout(:,1)];
            theta_hist = [theta_hist; NaN * xout(:,1)];
            theta_dot_hist = [theta_dot_hist; NaN * xout(:,1)];
            
            P.Visualize( tout, xout, current_mode );
            % Reset
            if ~isempty(event_id)
                xend = xout(end,:);
                x0 = Reset_F2S(xend,params);
                current_mode = 1;
            end
            
        otherwise
            error('Invalid Mode');
    end
end

% figure;
% plot(t_hist, x_dot_hist,'LineWidth',2);
% title('x dot');
% 
% figure;
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
% figure;
% plot(t_hist, l_hist, 'LineWidth', 2);
% title('l');
% 
% figure;
% plot(t_hist, l_dot_hist, 'LineWidth', 2);
% title('l dot');
