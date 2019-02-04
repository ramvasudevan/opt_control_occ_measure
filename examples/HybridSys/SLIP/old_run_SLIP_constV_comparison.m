% SLIP model with 3 modes.
% Goal: follow a constant-speed trajectory during t \in [0,T]
% running cost = ((v*t - 0.5) - x)^2, where v  = 0.1
% terminal cost = 0
% 
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
%-------------------------- Parameters for OCP ---------------------------%
%-------------------------------------------------------------------------%
T = 3;                          % time horizon
d = 8;                          % degree of relaxation
nmodes = 3;                     % number of modes

% Solver options
options.freeFinalTime = 0;      % fixed terminal time
options.withInputs = 1;         % control extraction?
options.svd_eps = 1e4;          % svd threshould for moment matrices

%-------------------------------------------------------------------------%
%---------------------------- Construct OCP ------------------------------%
%-------------------------------------------------------------------------%
% Define variables
t = msspoly( 't', 1 );
x = cell( nmodes, 1 );
u = cell( nmodes, 1 );
f = cell( nmodes, 1 );
g = cell( nmodes, 1 );
x0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
hU = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );

x{1} = msspoly( 'xa', 5 );
u{1} = msspoly( 'ua', 1 );
x{2} = msspoly( 'xb', 4 );
u{2} = u{1};                % dummy variable
x{3} = x{2};
u{3} = u{1};                % dummy variable

% Dynamics
f{1} = Stance_f_Approx( x{1}, params );
g{1} =  Stance_g_Approx( x{1}, params );
for i = 2 : 3
    f{i} = Flight_f( x{i}, params );
    g{i} = Flight_g( x{i}, params );
end

% Suppports, Reset Maps, and Cost Functions
domain = params.domain;
l0 = params.l0;
% Mode 1 : Stance
y = x{1};
hX{1} = [ domain{1}(:,2) - y;           % domain
          y - domain{1}(:,1) ];
hU{1} = u{1} * (1 - u{1});
R{1,2} = Reset_S2F_Approx(y,params);    % reset map
sX{1,2} = ...                           % guard
        [ -(l0 - y(1))^2;                   % l = l0
          y(2);                             % l_dot > 0
          hX{1};                            % G \subset X
          domain{2}(:,2) - R{1,2};          % Image(R(i,j)) \subset X_j
          R{1,2} - domain{2}(:,1) ];

h{1} = (0.1*T*t - 0.5 - y(5))^2;       % h = (0.1 * t - 1 - x)^2
H{1} = 0;

% Mode 2: Flight, y_dot > 0
y = x{2};
hX{2} = [ domain{2}(:,2) - y;           % domain
          y - domain{2}(:,1) ];
R{2,3} = y;                             % reset map
sX{2,3} = ...                           % guard
        [ -y(4)^2;                          % y_dot = 0
          hX{2};                            % G \subset X
          domain{3}(:,2) - R{2,3};          % Image(R(i,j)) \subset X_j
          R{2,3} - domain{3}(:,1) ];

h{2} = (0.1*T*t - 0.5 - y(1))^2;       % h = (0.1 * t - 1 - x)^2
H{2} = 0;

% Mode 3 : Flight 2
y = x{3};
hX{3} = [ domain{3}(:,2) - y;           % domain
          y - domain{3}(:,1) ];
R{3,1} = Reset_F2S_Approx(y,params);    % reset map
sX{3,1} = ...                           % guard
        [ -(y(3) - yR)^2;                   % y = yR
          hX{3};                          	% G \subset X
          domain{1}(:,2) - R{3,1};      	% Image(R(i,j)) \subset X_j
          R{3,1} - domain{1}(:,1) ];

h{3} = (0.1*T*t - 0.5 - y(1))^2;       % h = (0.1 * t - 1 - x)^2
H{3} = 0;

% Initial condition and Target Set
x0{3} = [ -0.5; 0.3; 0.20; 0 ];

% Target set is the entire space
hXT{1} = hX{1};
hXT{2} = hX{2};
hXT{3} = hX{3};

%-------------------------------------------------------------------------%
%-------------------------------- Solve ----------------------------------%
%-------------------------------------------------------------------------%
% P = SLIPPlot( 3, x0{3}, params ); 


% function handles
hf{1} = @(xx) Stance_f_Approx(xx,params);
hg{1} = @(xx) Stance_g_Approx(xx,params);
hf{2} = @(xx) Flight_f(xx,params);
hg{2} = @(xx) Flight_g(xx,params);
hf{3} = @(xx) Flight_f(xx,params);
hg{4} = @(xx) Flight_g(xx,params);

R_12 = @(xx) Reset_S2F_Approx( xx, params );
R_23 = @(xx) xx;
R_31 = @(xx) Reset_F2S_Approx( xx, params );

MaxTime = 1;
switch d
    case 4
        seq = [ 3; 1; 2; 3; 1 ];
    case 6
        seq = [ 3; 1; 2; 3 ];
    case 8
        seq = [ 3; 1; 2; 3; 1 ];
end
t_hist = 0;
state_hist = nan(1,9);
u_hist = 0;
pval = 0;
total_time = 0;

for i = 1 : length( seq )
    Tleft = T - t_hist(end);
    
    cmode = seq( i );
    xvar = x{ cmode };
    uvar = u{ cmode };
    if i == 1
        cx0 = x0{ cmode };
    end
    chX = hX{ cmode };
%     ch = h{ cmode } * Tleft;
    switch cmode
        case 1
            ch = ( 0.1 * ( Tleft * t + (T-Tleft) ) - 0.5 - xvar(5) )^2;    % h = (0.1 * t - 0.5 - x)^2
        otherwise
            ch = ( 0.1 * ( Tleft * t + (T-Tleft) ) - 0.5 - xvar(1) )^2;    % h = (0.1 * t - 0.5 - x)^2
    end
    
    if i == length(seq)
        chXT = hXT{ cmode };
        cH = H{ cmode };
        options.freeFinalTime = 0;
    else
        chXT = sX{ seq(i), seq(i+1) };
        cH = msspoly( 0 );
        options.freeFinalTime = 1;
    end
    
%     Tleft = T;
    if cmode == 1
        options.withInputs = 1;
    else
        options.withInputs = 0;
    end
    
%     if cmode == 1
    [out] = OCPDualSolver( t, xvar, uvar, Tleft*f{cmode}, Tleft*g{cmode}, cx0, chX, chXT, ch, cH, d, options );
    pval = pval + Tleft * out.pval;
    total_time = total_time + out.time;
%     end
    
    
    MaxTime = (T - t_hist(end)) / Tleft;
    % integrate forward
    switch cmode
        case 1          % stance
            controller = @(tt,xx) max(0,min(1,double(subs(out.u{1}, [t;xvar], [tt;xx]))));
            ode_options = odeset('Events',@(tt,xx) EvtFunc_S2F_scaled(tt,xx,params));
            ode_options = odeset(ode_options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) Tleft * ( hf{1}(xx) + hg{1}(xx) * controller(tt,xx) ), ...
                             (0 : 1e-2 : MaxTime), cx0, ode_options);
            % Reset
            if ~isempty(event_id)
                if event_id(end) ~= 1
                    error('Something wrong.');
                else
                    xend = xout(end,:);
                    cx0 = R_12(xend);
                end
            end
            % Save
%             P.Visualize( tout * Tleft + t_hist(end), xout, 1 );
            
            t_hist = [ t_hist; t_hist(end) + tout * Tleft ];
            mat = [ eye(5), nan*ones(5,3) ];
            tmp = xout*mat;
            tmp(:,7) = tmp(:,1) .* cos(tmp(:,3));
            state_hist = [ state_hist; tmp, 1*ones(length(tout),1) ];
            
%             P.Visualize( tout, xout, 1 );
        case 2          % flight 1, ydot > 0
            ode_options = odeset('Events',@(tt,xx) EvtFunc_F1_scaled(tt,xx,params));
            ode_options = odeset(ode_options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) Tleft * ( hf{2}(xx) ), ...
                             (0 : 1e-2 :  MaxTime), cx0, ode_options);
            % Reset
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    error('Something wrong 2.');
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    cx0 = R_23(xend);
                    cx0 = cx0(:);
                    current_mode = 3;
                end
            end
%             if xs(3)<yR
%                 xs = R_31(xs);
%                 current_mode = 1;
%             end
%             current_time = tout(end);
            % Plot
%             P.Visualize( tout * Tleft + t_hist(end), xout, 2 );
            t_hist = [t_hist; t_hist(end) + tout * Tleft];
            mat = [ nan*ones(4,4), eye(4) ];
            state_hist = [ state_hist; xout*mat, 2*ones(length(tout),1) ];
            
%             P.Visualize( tout, xout, 2 );
        case 3          % flight 2, ydot < 0
            ode_options = odeset('Events',@(tt,xx) EvtFunc_F2S_scaled(tt,xx,params));
            ode_options = odeset(ode_options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) Tleft * ( hf{3}(xx) ), ...
                             (0 : 1e-2 : MaxTime), cx0, ode_options);
            % Reset
%             previous_mode = current_mode;
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    current_mode = 0;
                    error('Something wrong 3.');
                else
                    xend = xout(end,:);
                    cx0 = R_31(xend);
                    current_mode = 1;
                end
            end
            current_time = tout(end);
            % Plot
%             P.Visualize( tout * Tleft + t_hist(end), xout, 3 );
            
            t_hist = [t_hist; t_hist(end) + tout * Tleft];
            mat = [ nan*ones(4,4), eye(4) ];
            state_hist = [ state_hist; xout*mat, 3*ones(length(tout),1) ];
            
%             P.Visualize( tout, xout, 3 );
    end
    
end


%% -------------------------------------------------------------------------%
%-------------------------------- Cost -----------------------------------%
%-------------------------------------------------------------------------%
t_hist = t_hist( 2:end );
state_hist = state_hist( 2:end, : );

cost = 0;
for i = 1 : length(t_hist)-1
    
    current_mode = state_hist(i,end);
    dt = t_hist(i+1) - t_hist(i);
    cost = cost + dt * ( 0.1 * t_hist(i+1) - 0.5 - state_hist(i+1,5) )^2;
    
%     if current_mode == 1
%         
%         state = ( state_hist( i, 1:5 ) )';
%         dt = t_hist(i+1) - t_hist(i);
%         cost = cost + dt * double( subs(h{current_mode}, [ t; x{current_mode} ], [ t_hist(i); state ] ) );
%        
%     elseif current_mode > 1
%     
%         state = ( state_hist( i, 5:8 ) )';
%         dt = t_hist(i+1) - t_hist(i);
%         cost = cost + dt * double( subs(h{current_mode}, [ t; x{current_mode} ], [ t_hist(i); state ] ) );
%         
%     else
%         
%         disp('terminate');
%         
%     end
end

disp(['total time = ', num2str(total_time)]);
disp(['pval = ', num2str(pval)]);
disp(['cost = ', num2str(cost)]);

save(['Rebuttal_SLIP_constV_d', num2str(d),'_T3']);

