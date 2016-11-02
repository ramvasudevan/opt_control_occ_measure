% Simulation : SLIP
% Stance & Flight
% Simulation with 3 modes. I understand that the dynamics for F1 and F2 are
% the same, but just to see all the guards and reset maps make sense.
% 
% For all the cases I've tested on, the results matched the ones given by
% run_sim.m well.

% clear;
% close all;

addpath('Utils');
addpath('Dynamics');
addpath('EventFcns');
addpath('Dynamics3');

params = SLIPParams;
controller = @(x) -1;
% current_mode = 3;       % 1 = Stance; 2 = Flight, under; 3 = Flight, above
% x0 = [ 0; 1.5; 1.2; 0 ];

current_mode = 2;
x0 = [ 0; 1.5; 0.6; 2 ];

MaxTime = 6;
tspan = 0 : 0.01 : MaxTime;
current_time = 0;

% P = SLIPPlot( current_mode, x0, params );
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
            options = odeset('Events',@EvtFunc_S);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Stance_simp_f(x,params) + ...
                             Stance_simp_g(x,params) * controller(x), ...
                             (current_time : 0.01 : MaxTime), x0, options);
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
            
%             P.Visualize( tout, xout, current_mode );
            % Reset
            if ~isempty(event_id)
                xend = xout(end,:);
                x0 = Reset_S2F(xend,params);
                if (x0(3) <= 0)
                    current_mode = 2;
                else
                    current_mode = 3;
                end
            end
        case 2      % Flight 1 (under ground)
            options = odeset('Events',@EvtFunc_F1);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Flight_Approx_f(x,params) + ...
                             Flight_Approx_g(x,params) * controller(x), ...
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
            
            
%             P.Visualize( tout, xout, current_mode );
            % Reset
            if ~isempty(event_id)
                xend = xout(end,:);
                x0 = Reset_F12F2(xend,params);
                current_mode = 3;
            end
        case 3      % Flight 2 (above ground)
            options = odeset('Events',@EvtFunc_F2);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Flight_Approx_f(x,params) + ...
                             Flight_Approx_g(x,params) * controller(x), ...
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
            
%             P.Visualize( tout, xout, current_mode );
            % Reset
            if ~isempty(event_id)
                xend = xout(end,:);
                x0 = Reset_F22S(xend,params);
                current_mode = 1;
            end
            
        otherwise
            error('Invalid Mode');
    end
end

% figure(1);
% plot(t_hist, x_hist, 'LineWidth', 3);
% figure(2);
% plot(t_hist, theta_hist, 'LineWidth', 3);
% figure(3);
% plot(t_hist, y_hist, 'LineWidth', 3);

figure;
plot(t_hist, l_hist, 'LineWidth', 3);
