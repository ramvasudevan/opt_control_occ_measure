% Simulation : SLIP
% Stance & Flight
% Simulation with 3 modes. Even though the dynamics for F1 and F2 are
% the same, we just want to check all the guards and reset maps make sense.
% 
% For all the cases I've tested on, the results matched the ones given by
% run_sim.m well.

clear;
close all;

% addpath('Utils');
% addpath('PolyDynamics');
% addpath('TrueDynamics');
% addpath('Dynamics3');

params = SLIPParams;
controller = @(xx) 0;
% current_mode = 3;       % 1 = Stance; 2 = Flight, under; 3 = Flight, above
% x0 = [ 0; 1.5; 1.2; 0 ];

current_mode = 3;
x_initial = [ -1; 0.3; 0.20; 0 ];


x0 = x_initial;
Target = -0.54;

opt = [ ...     % 1 = actual, 2 = taylor expansion
        1;      % Dynamics
        1;      % Guard
        1;      % Reset map
       ];
flag_draw = 1;

MaxTime = 4;
current_time = 0;

if flag_draw, P = SLIPPlot( current_mode, x0, params ); end
t_hist = [];
mode_hist = [];
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
                             Stance_g(x) * controller(x), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            else
                [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Stance_f_Approx(x) + ...
                             Stance_g_Approx(x) * controller(x), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            end
            current_time = tout(end);
            t_hist = [t_hist; tout];
            mode_hist = [mode_hist; ones(length(tout),1)*current_mode];
            x_hist = [x_hist; xout(:,5)];
            x_dot_hist = [x_dot_hist; NaN * xout(:,1)];
            y_hist = [y_hist; NaN * xout(:,1)];
            y_dot_hist = [y_dot_hist; NaN * xout(:,1)];
            l_hist = [l_hist; xout(:,1)];
            l_dot_hist = [l_dot_hist; xout(:,2)];
            theta_hist = [theta_hist; xout(:,3)];
            theta_dot_hist = [theta_dot_hist; xout(:,4)];
            
            if flag_draw
                P.Visualize( tout, xout, current_mode );
            end
            % Reset
            if ~isempty(event_id)
                if opt(3)
                    x0 = Reset_S2F(xout(end,:),params);
                else
                    x0 = Reset_S2F_Approx(xout(end,:),params);
                end
                current_mode = 2;
            end
        case 2      % Flight 1 (going up)
            options = odeset('Events',@EvtFunc_F1);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Flight_f(x,params) + ...
                             Flight_g(x,params) * controller(x), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            current_time = tout(end);
            t_hist = [t_hist; tout];
            mode_hist = [mode_hist; ones(length(tout),1)*current_mode];
            x_hist = [x_hist; xout(:,1)];
            x_dot_hist = [x_dot_hist; xout(:,2)];
            y_hist = [y_hist; xout(:,3)];
            y_dot_hist = [y_dot_hist; xout(:,4)];
            l_hist = [l_hist; NaN * xout(:,1)];
            l_dot_hist = [l_dot_hist; NaN * xout(:,1)];
            theta_hist = [theta_hist; NaN * xout(:,1)];
            theta_dot_hist = [theta_dot_hist; NaN * xout(:,1)];
            
            if flag_draw
                P.Visualize( tout, xout, current_mode );
            end
            % Reset
            if ~isempty(event_id)
                xend = xout(end,:);
                x0 = xend;
                current_mode = 3;
            end
        case 3      % Flight 2 (going down)
            options = odeset('Events',@EvtFunc_F2S);
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(t,x) Flight_f(x,params) + ...
                             Flight_g(x,params) * controller(x), ...
                             (current_time : 0.01 : MaxTime), x0, options);
            current_time = tout(end);
            t_hist = [t_hist; tout];
            mode_hist = [mode_hist; ones(length(tout),1)*current_mode];
            x_hist = [x_hist; xout(:,1)];
            x_dot_hist = [x_dot_hist; xout(:,2)];
            y_hist = [y_hist; xout(:,3)];
            y_dot_hist = [y_dot_hist; xout(:,4)];
            l_hist = [l_hist; NaN * xout(:,1)];
            l_dot_hist = [l_dot_hist; NaN * xout(:,1)];
            theta_hist = [theta_hist; NaN * xout(:,1)];
            theta_dot_hist = [theta_dot_hist; NaN * xout(:,1)];
            
            if flag_draw
                P.Visualize( tout, xout, current_mode );
            end
            % Reset
            if ~isempty(event_id)
                if opt(3)
                    x0 = Reset_F2S(xout(end,:),params);
                else
                    x0 = Reset_F2S_Approx(xout(end,:),params);
                end
                current_mode = 1;
            end
            
        otherwise
            error('Invalid Mode');
    end
end

state_hist = [ l_hist, l_dot_hist, theta_hist, theta_dot_hist,...
                       x_hist, x_dot_hist, y_hist, y_dot_hist, mode_hist ];

mode_hist = mode_hist(x_hist<Target);
t_hist = t_hist(x_hist<Target);
state_hist = state_hist(x_hist<Target,:);
idx_mat = diff(mode_hist);
idx = find( idx_mat ~= 0 );


% Construct 'guess'
guess.phase(1).time = 0;
guess.phase(1).state = x_initial';

for i = 1 : length(idx)
    
    current_mode = mode_hist(idx(i));
    guess.phase(i).time = [ guess.phase(i).time; t_hist(idx(i)) ];
    guess.phase(i).integral = guess.phase(i).time(2) - guess.phase(i).time(1);
    guess.phase(i+1).time = t_hist(idx(i)+1);
    switch current_mode
        case 1
            guess.phase(i).state = [ guess.phase(i).state; state_hist(idx(i), 1:5) ];
            guess.phase(i).control = [ 0; 0 ];
            guess.phase(i+1).state = state_hist(idx(i)+1, 5:8);
        case 2
            guess.phase(i).state = [ guess.phase(i).state; state_hist(idx(i), 5:8) ];
            guess.phase(i+1).state = state_hist(idx(i)+1, 5:8);
        case 3
            guess.phase(i).state = [ guess.phase(i).state; state_hist(idx(i), 5:8) ];
            guess.phase(i+1).state = state_hist(idx(i)+1, 1:5);
        otherwise
            error('Something wrong.');
    end
end

current_mode = mode_hist(end);
i = length(idx)+1;
guess.phase(i).time = [ guess.phase(i).time; t_hist(end) ];
switch current_mode
    case 1
        guess.phase(i).state = [ guess.phase(i).state; state_hist(end,1:5) ];
        guess.phase(i).control = [ 0; 0 ];
    case {2,3}
        guess.phase(i).state = [ guess.phase(i).state; state_hist(end,5:8) ];
end
guess.phase(i).integral = guess.phase(i).time(2) - guess.phase(i).time(1);


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
