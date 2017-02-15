% Generate sample trajectory

clear;
close all;
%-------------------------------------------------------------------------%
%-------------------- All Physical Parameters for SLIP -------------------%
%-------------------------------------------------------------------------%
params = struct();

params.m        = 1;            % mass
params.k        = 6;            % spring constant
params.g        = 0.2;          % gravitational acceleration
params.l0       = 0.2;          % maximum leg length

params.alpha    = pi/6;         % leg angle in flight phase
params.umax     = 0.1;          % upper bound of input

%-------------------------------------------------------------------------%
%-------------- Domains (Defined by Upper and Lower Bounds) --------------%
%-------------------------------------------------------------------------%
yR  = params.l0 * cos(params.alpha);        % touch-down height
params.yR = yR;

% Mode 1: stance
% state = ( l, l_dot, theta, theta_dot, x )
params.domain{1} =...
        [ 0.1, 0.2;         % l         - leg length
         -0.3, 0.3;         % l_dot     - time derivative of l
           -1, 1;           % theta     - leg angle
           -3, 0;           % theta_dot - time derivative of theta
           -1, 1 ];         % x         - horizontal displacement

% Mode 2: flight, vertical velocity is positive (upwards)
% state = ( x, x_dot, y, y_dot )
params.domain{2} = ...
         [ -1, 1;           % x         - horizontal displacement
            0, 0.5;         % x_dot     - time derivative of x
         0.15, 0.5;         % y         - vertical displacement
            0, 1 ];         % y_dot     - time derivative of y

% Mode 3: flight, vertical velocity is negative (downwards)
% state = ( x, x_dot, y, y_dot )
params.domain{3} = ...
         [ -1, 1;           % x         - horizontal displacement
            0, 0.5;         % x_dot     - time derivative of x
           yR, 0.5;         % y         - vertical displacment
           -1, 0 ];         % y_dot     - time derivative of y

%-------------------------------------------------------------------------%
%------------------- Simulate forward to get trajectory ------------------%
%-------------------------------------------------------------------------%

current_mode = 3;
xs = [ -1; 0.3; 0.20; 0 ];
T = 2.7;

previous_mode = 0;

opt = [ ...     % 1 = actual, 0 = taylor expansion
        0;      % Dynamics
        0;      % Guard
        0;      % Reset map
       ];

MaxTime = 1;
f = cell(3,1);
g = cell(3,1);
% Dynamics
if opt(1)
    f{1} = @(xx) T * Stance_f(xx,params);
    g{1} = @(xx) T * Stance_g(xx,params);
else
    f{1} = @(xx) T * Stance_f_Approx(xx,params);
    g{1} = @(xx) T * Stance_g_Approx(xx,params);
end
f{2} = @(xx) T * Flight_f(xx,params);
g{2} = @(xx) T * Flight_g(xx,params);
f{3} = @(xx) T * Flight_f(xx,params);
g{4} = @(xx) T * Flight_g(xx,params);
% Reset map
if opt(3)
    R_12 = @(xx) Reset_S2F(xx);
    R_23 = @(xx) xx;
    R_31 = @(xx) Reset_F2S(xx);
else
    R_12 = @(xx) Reset_S2F_Approx( xx, params );
    R_23 = @(xx) xx;
    R_31 = @(xx) Reset_F2S_Approx( xx, params );
end

controller = @(tt,xx) 0.5;

current_time = 0;

% if exist('mycolor','var') && exist('mythickness', 'var')
% 	P = SLIPPlot( current_mode, xs, params, mycolor, mythickness );
% else
% 	P = SLIPPlot( current_mode, xs, params ); 
% end
state_hist = [];        % [ l, ldot, theta, thetadot, x, xdot, y, ydot, mode ]
t_hist = [];

while current_time < MaxTime - 0.01
    disp(current_mode);
    switch current_mode
        case 1      % Stance
            options = odeset('Events',@(tt,xx) EvtFunc_S2F_scaled(tt,xx,params));
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
                [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{1}(xx) + g{1}(xx) * controller(tt,xx) ), ...
                             (current_time : 1e-3 : MaxTime), xs, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                if event_id(end) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    xs = R_12(xend);
                    current_mode = 2;
                end
            end
            current_time = tout(end);
            % Plot
            t_hist = [ t_hist; tout ];
            mat = [ eye(5), nan*ones(5,3) ];
            tmp = xout*mat;
            tmp(:,7) = tmp(:,1) .* cos(tmp(:,3));
            state_hist = [ state_hist; tmp, 1*ones(length(tout),1) ];
            
        case 2      % Flight 1 (under ground)
            options = odeset('Events',@(tt,xx) EvtFunc_F1_scaled(tt,xx,params));
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{2}(xx) ), ...
                             (current_time : 1e-3 :  MaxTime), xs, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    xs = R_23(xend);
                    current_mode = 3;
                end
            end
            if xs(3)<yR
                xs = R_31(xs);
                current_mode = 1;
            end
            current_time = tout(end);
            % Plot
            t_hist = [t_hist; tout];
            mat = [ nan*ones(4,4), eye(4) ];
            state_hist = [ state_hist; xout*mat, 2*ones(length(tout),1) ];
            
        case 3      % Flight 2 (above ground)
            options = odeset('Events',@(tt,xx) EvtFunc_F2S_scaled(tt,xx,params));
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{3}(xx) ), ...
                             (current_time : 1e-3 : MaxTime), xs, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    xs = R_31(xend);
                    current_mode = 1;
                end
            end
            current_time = tout(end);
            % Plot
            t_hist = [t_hist; tout];
            mat = [ nan*ones(4,4), eye(4) ];
            state_hist = [ state_hist; xout*mat, 3*ones(length(tout),1) ];
            
        case 0
            break;
            
        otherwise
            error('Invalid Mode');
    end
end

t_hist = t_hist( state_hist(:,5)<=10 );
state_hist = state_hist( state_hist(:,5)<=10, : );
l_hist          = state_hist(:,1);
l_dot_hist      = state_hist(:,2);
theta_hist      = state_hist(:,3);
theta_dot_hist  = state_hist(:,4);
x_hist          = state_hist(:,5);
x_dot_hist      = state_hist(:,6);
y_hist          = state_hist(:,7);
y_dot_hist      = state_hist(:,8);
mode_hist       = state_hist(:,9);

%% Plot
if ~exist('mycolor','var')
    mycolor = [ 0, 0.4470, 0.7410 ];
end
h_2d = gca;

hold on;
idx_mat = mode_hist(2:end) - mode_hist(1:end-1);
idx = [ find( idx_mat > 0 ); find( idx_mat < 0 )+1];
idx = [1; idx];
springcoord( [ 0, 0 ] , [ 0, params.l0 ], 5, 0.3, 0.04);

plot( h_2d, x_hist, y_hist, '-.', 'color', mycolor, 'LineWidth', 1);

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

plot([-1,1],[0,0],'k');

axis equal
ylim([-0.1, 0.3]);
xlim([-0.9,-0.22]);
axis off