% Scaled dynamics - to compare with the result given by run_sim.m
% The only thing that's different is the dynamics is scaled down to [-1,1]
% And the trajectories are plotted after being scaled back.

% Simulation : SLIP
% Stance & Flight

clear;
% close all;

addpath('Utils');
addpath('Dynamics');
addpath('EventFcns');

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
y_hist = [];
theta_hist = [];

% Scale dynamics
domain_size = [ 0.5, 1;
                -5, 5;
                -pi/3, pi/3;
                -2*pi/3, 0;
                0, 5 ];
scale_x = (domain_size(:,2) - domain_size(:,1)) / 2;
trans_x = mean(domain_size,2);

fs = @(xx) rescale_dynamics(@(x) Stance_simp_f(x,params), xx, scale_x, trans_x, scale_x );
gs = @(xx) rescale_dynamics(@(x) Stance_simp_g(x,params), xx, scale_x, trans_x, scale_x );
Rs_S2F = @(xx) rescale_reset(@(x) Reset_S2F(x), xx, scale_x, trans_x, 1, 0);
Rs_F2S = @(xx) rescale_reset(@(x) Reset_F2S(x), xx, 1, 0, scale_x, trans_x);

while current_time < MaxTime - 0.1
    disp(current_mode);
    switch current_mode
        case 1      % Stance
            options = odeset('Events',@EvtFunc_S2F);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) fs(x) + ...
                             gs(x) * controller(x), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            current_time = tout(end);
            t_hist = [t_hist; tout];
            x_hist = [x_hist; xout(:,5)*scale_x(5) + trans_x(5)];
            y_hist = [y_hist; NaN*xout(:,1)];
            theta_hist = [theta_hist; xout(:,3)*scale_x(3) + trans_x(3)];
            
%             P.Visualize( tout, xout, current_mode );
            % Reset
            if ~isempty(event_id)
                x0 = Rs_S2F(xout(end,:)');
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
            
%             P.Visualize( tout, xout, current_mode );
            % Reset
            if ~isempty(event_id)
                x0 = Rs_F2S(xout(end,:)');
                current_mode = 1;
            end
        otherwise
            error('Invalid Mode');
    end
end

figure(1);
title('x');
hold on;
plot(t_hist, x_hist,'LineWidth',3);

figure(2);
title('theta');
hold on;
plot(t_hist, theta_hist, 'LineWidth', 3);

figure(3);
title('y');
hold on;
plot(t_hist, y_hist, 'LineWidth', 3);