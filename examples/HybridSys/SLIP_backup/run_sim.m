% Simulation : SLIP
% Stance & Flight

clear;
close all;

addpath('Utils');
addpath('Dynamics');
addpath('EventFcns');

params = SLIPParams;
controller = @(x) 0;
% current_mode = 3;       % 1 = Stance; 2 = Flight, under; 3 = Flight, above
% x0 = [ 0; 1.5; 1.2; 0 ];

current_mode = 2;
x0 = [ 0; 1.5; 1.2; 0 ];

MaxTime = 6;
tspan = 0 : 0.01 : MaxTime;
current_time = 0;

P = SLIPPlot( current_mode, x0, params );
t_hist = [];
x_hist = [];
y_hist = [];
theta_hist = [];

while current_time < MaxTime - 0.1
    disp(current_mode);
    switch current_mode
        case 1      % Stance
            options = odeset('Events',@EvtFunc_S2F);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Stance_simp_f(x,params) + ...
                             Stance_simp_g(x,params) * controller(x), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            current_time = tout(end);
            t_hist = [t_hist; tout];
            x_hist = [x_hist; xout(:,5)];
            y_hist = [y_hist; NaN*xout(:,1)];
            theta_hist = [theta_hist; xout(:,3)];
            
            P.Visualize( tout, xout, current_mode );
            % Reset
            if ~isempty(event_id)
                x0 = Reset_S2F(xout(end,:),params);
                current_mode = 2;
            end
        case {2,3}      % Flight
            options = odeset('Events',@EvtFunc_F2S);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Flight_Approx_f(x,params) + ...
                             Flight_Approx_g(x,params) * controller(x), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            current_time = tout(end);
            t_hist = [t_hist; tout];
            x_hist = [x_hist; xout(:,1)];
            y_hist = [y_hist; xout(:,3)];
            theta_hist = [theta_hist; NaN * xout(:,1)];
            
            P.Visualize( tout, xout, current_mode );
            % Reset
            if ~isempty(event_id)
                x0 = Reset_F2S(xout(end,:),params);
                current_mode = 1;
            end
        otherwise
            error('Invalid Mode');
    end
end

figure(2);
title('x');
hold on;
plot(t_hist, x_hist,'LineWidth',7);

figure(3);
title('theta');
hold on;
plot(t_hist, theta_hist, 'LineWidth', 7);

figure(4);
title('y');
hold on;
plot(t_hist, y_hist, 'LineWidth', 7);