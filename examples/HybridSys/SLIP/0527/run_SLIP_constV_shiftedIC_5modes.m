% SLIP model with 3 modes.
% Goal: Follow a constant-speed trajectory during t \in [0,T]
% running cost = ((v*t - 1) - x)^2
% terminal cost = 0
% 
% IC shifted to -0.5

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
           -3, 3;           % theta_dot - time derivative of theta
           -1, 1 ];         % x         - horizontal displacement

% Mode 2: flight, vertical velocity is positive (upwards), xdot > 0
% state = ( x, x_dot, y, y_dot )
params.domain{2} = ...
         [ -1, 1;           % x         - horizontal displacement
            0, 0.5;         % x_dot     - time derivative of x
         0.15, 0.5;         % y         - vertical displacement
            0, 1 ];         % y_dot     - time derivative of y

% Mode 3: flight, vertical velocity is negative (downwards), xdot > 0
% state = ( x, x_dot, y, y_dot )
params.domain{3} = ...
         [ -1, 1;           % x         - horizontal displacement
            0, 0.5;         % x_dot     - time derivative of x
           yR, 0.5;         % y         - vertical displacment
           -1, 0 ];         % y_dot     - time derivative of y

% Mode 4: flight, vertical velocity is positive (upwards), xdot < 0
% state = ( x, x_dot, y, y_dot )
params.domain{4} = ...
         [ -1, 1;           % x         - horizontal displacement
           -0.5, 0;         % x_dot     - time derivative of x
         0.15, 0.5;         % y         - vertical displacement
            0, 1 ];         % y_dot     - time derivative of y

% Mode 5: flight, vertical velocity is negative (downwards), xdot < 0
% state = ( x, x_dot, y, y_dot )
params.domain{5} = ...
         [ -1, 1;           % x         - horizontal displacement
           -0.5, 0;         % x_dot     - time derivative of x
           yR, 0.5;         % y         - vertical displacment
           -1, 0 ];         % y_dot     - time derivative of y

%-------------------------------------------------------------------------%
%-------------------------- Parameters for OCP ---------------------------%
%-------------------------------------------------------------------------%
d = 6;              % degree of relaxation
T = 2.5;              % time horizon
v = 0.35;
nmodes = 5;         % number of modes

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
x{4} = x{2};
u{4} = u{1};                % dummy variable
x{5} = x{2};
u{5} = u{1};                % dummy variable

% Dynamics
f{1} = T * Stance_f_Approx( x{1}, params );
g{1} = T * Stance_g_Approx( x{1}, params );
for i = 2 : 5
    f{i} = T * Flight_f( x{i}, params );
    g{i} = T * Flight_g( x{i}, params );
end

% Cost functions
costS = (v*T*t - 0.5 - x{1}(5))^2 * T;     % running cost for stance modes
costF = (v*T*t - 0.5 - x{2}(1))^2 * T;     % running cost for flight modes

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

R{1,4} = Reset_S2F_Approx(y,params);    % reset map
sX{1,4} = ...                           % guard
        [ -(l0 - y(1))^2;                   % l = l0
          y(2);                             % l_dot > 0
          hX{1};                            % G \subset X
          domain{4}(:,2) - R{1,4};          % Image(R(i,j)) \subset X_j
          R{1,4} - domain{4}(:,1) ];

h{1} = costS;         % h = (0.3 * t/T - 1 - x)^2
H{1} = 0;

% Mode 2: Flight, y_dot > 0, x_dot > 0
y = x{2};
hX{2} = [ domain{2}(:,2) - y;           % domain
          y - domain{2}(:,1) ];
R{2,3} = y;                             % reset map
sX{2,3} = ...                           % guard
        [ -y(4)^2;                          % y_dot = 0
          hX{2};                            % G \subset X
          domain{3}(:,2) - R{2,3};          % Image(R(i,j)) \subset X_j
          R{2,3} - domain{3}(:,1) ];

R{2,4} = y;                             % reset map
sX{2,4} = ...                           % guard
        [ -y(2)^2;                          % x_dot = 0
          hX{2};                            % G \subset X
          domain{4}(:,2) - R{2,4};          % Image(R(i,j)) \subset X_j
          R{2,4} - domain{4}(:,1) ];

h{2} = costF;         % h = (0.3 * t/T - 1 - x)^2
H{2} = 0;

% Mode 3 : Flight 2, y_dot < 0, x_dot > 0
y = x{3};
hX{3} = [ domain{3}(:,2) - y;           % domain
          y - domain{3}(:,1) ];
R{3,1} = Reset_F2S_Approx(y,params);    % reset map
sX{3,1} = ...                           % guard
        [ -(y(3) - yR)^2;                   % y = yR
          hX{3};                          	% G \subset X
          domain{1}(:,2) - R{3,1};      	% Image(R(i,j)) \subset X_j
          R{3,1} - domain{1}(:,1) ];

R{3,5} = y;
sX{3,5} = ...                           % guard
        [ -y(2)^2;                          % x_dot = 0
          hX{3};                          	% G \subset X
          domain{5}(:,2) - R{3,5};      	% Image(R(i,j)) \subset X_j
          R{3,5} - domain{5}(:,1) ];

h{3} = costF;         % h = (0.3 * t/T - 1 - x)^2
H{3} = 0;

% Mode 4: Flight, y_dot > 0, x_dot < 0
y = x{4};
hX{4} = [ domain{4}(:,2) - y;           % domain
          y - domain{4}(:,1) ];
R{4,5} = y;                             % reset map
sX{4,5} = ...                           % guard
        [ -y(4)^2;                          % y_dot = 0
          hX{4};                            % G \subset X
          domain{5}(:,2) - R{4,5};          % Image(R(i,j)) \subset X_j
          R{4,5} - domain{5}(:,1) ];

R{4,2} = y;                             % reset map
sX{4,2} = ...                           % guard
        [ -y(2)^2;                          % x_dot = 0
          hX{2};                            % G \subset X
          domain{2}(:,2) - R{4,2};          % Image(R(i,j)) \subset X_j
          R{4,2} - domain{2}(:,1) ];

h{4} = costF;         % h = (0.3 * t/T - 1 - x)^2
H{4} = 0;

% Mode 5 : Flight 2, y_dot < 0, x_dot < 0
y = x{5};
hX{5} = [ domain{3}(:,2) - y;           % domain
          y - domain{3}(:,1) ];
R{5,1} = Reset_F2S_Approx(y,params);    % reset map
sX{5,1} = ...                           % guard
        [ -(y(3) - yR)^2;                   % y = yR
          hX{5};                          	% G \subset X
          domain{1}(:,2) - R{5,1};      	% Image(R(i,j)) \subset X_j
          R{5,1} - domain{1}(:,1) ];

R{5,3} = y;
sX{5,3} = ...                           % guard
        [ -y(2)^2;                          % x_dot = 0
          hX{5};                          	% G \subset X
          domain{3}(:,2) - R{5,3};      	% Image(R(i,j)) \subset X_j
          R{5,3} - domain{3}(:,1) ];

h{5} = costF;         % h = (0.3 * t/T - 1 - x)^2
H{5} = 0;

% Initial condition and Target Set
x0{3} = [ -0.5; 0.3; 0.20; 0 ];

% Target set is the entire space
hXT{1} = hX{1};
hXT{2} = hX{2};
hXT{3} = hX{3};

% % Scale running cost
% h{1} = h{1} * 1e6;
% h{2} = h{2} * 1e6;
% h{3} = h{3} * 1e6;
%-------------------------------------------------------------------------%
%-------------------------------- Solve ----------------------------------%
%-------------------------------------------------------------------------%
[out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);

disp(['LMI ' int2str(d) ' lower bound = ' num2str(out.pval)]);

%-------------------------------------------------------------------------%
%-------------------------------- PLot -----------------------------------%
%-------------------------------------------------------------------------%
PlotRelaxedControl;
GenerateGuess;

