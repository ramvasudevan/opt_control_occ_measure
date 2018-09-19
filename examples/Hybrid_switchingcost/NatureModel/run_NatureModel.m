%        theta = positive
% o     o     o
%  \    |--->/
%   \   ^   / l
%    \  F  /
%     \ | /
%      \|/
% ============

%-------------------------------------------------------------------------%
%------------------------ All Physical Parameters ------------------------%
%-------------------------------------------------------------------------%
params = struct();

params.m        = 0.2;            % mass
params.g        = 0.2;          % gravitational acceleration
params.l0       = 0.5;          % leg length at touch-down
params.lmax     = 1;            % maximum leg length

params.alpha    =-pi/6;         % leg angle in flight phase = -30 degrees
params.umax     = 1;          % upper bound of input

%-------------------------------------------------------------------------%
%-------------- Domains (Defined by Upper and Lower Bounds) --------------%
%-------------------------------------------------------------------------%
yR  = params.l0 * cos(params.alpha);        % touch-down height
params.yR = yR;
lmax = params.lmax;


% Mode 1: stance, y <= yR
% state = ( l, l_dot, theta, theta_dot )
params.domain{1} =...
        [ 0.1, lmax;        % l         - leg length
         -0.5, 0.5;         % l_dot     - time derivative of l
         -1.2, 1.2;           % theta     - leg angle
            -0.1, 2 ];         % theta_dot - time derivative of theta

% Mode 2: stance, y >= yR
% state = ( l, l_dot, theta, theta_dot )
params.domain{2} =...
        [ 0.1, lmax;        % l         - leg length
         -0.5, 0.5;         % l_dot     - time derivative of l
         -1.2, 1.2;           % theta     - leg angle
            -0.1, 2 ];         % theta_dot - time derivative of theta

%-------------------------------------------------------------------------%
%-------------------------- Parameters for OCP ---------------------------%
%-------------------------------------------------------------------------%
d = 6;              % degree of relaxation
T = 3;              % time horizon
nmodes = 2;         % number of modes

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
hX0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
hU = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );
c = cell( nmodes, 1 );

xvar = msspoly('x', 4);
uvar = msspoly('u', 1);

x{1} = xvar;
u{1} = uvar;
x{2} = xvar;
u{2} = uvar;

% Dynamics
for i = 1 : 2
    f{i} = T * Swing_f_poly( x{i}, params );
    g{i} = T * Swing_g_poly( x{i}, params );
end

% Suppports, Reset Maps, and Cost Functions
domain  = params.domain;
l0      = params.l0;
umax    = params.umax;
al      = params.alpha;
polysin = @(ang) ang - ang.^3/6 + ang.^5/120;
polycos = @(ang) 1 - ang.^2/2 + ang.^4/24;

y = xvar(1) * polycos(xvar(3));             % y = l * cos(theta)
ydot = xvar(2) * polycos(xvar(3)) - xvar(1) * xvar(4) * polysin(xvar(3));   % y_dot = l_dot * cos(theta) - l * theta_dot * sin(theta)

d_des = 0.6;        % desired step length

% Mode 1 : Stance (y <= yR)
hX{1} = [ domain{1}(:,2) - xvar;            % domain
          xvar - domain{1}(:,1);
          yR - y ];
hU{1} = uvar * (umax - uvar);
R{1,2} = xvar;                              % reset map
sX{1,2} = ...                               % guard
        [ y - yR;                       % y = yR
          yR - y;
          ydot;                         % ydot > 0
          hX{1} ];                      % G \subset X
c{1,2} = msspoly(0);
h{1} = uvar^2;
H{1} = msspoly(0);

% Mode 2 : Stance (y >= yR)
hX{2} = [ domain{1}(:,2) - xvar;            % domain
          xvar - domain{1}(:,1);
          y - yR ];
hU{2} = uvar * (umax - uvar);
R{2,1} = Reset_S2S_poly(xvar,params);       % reset map
sX{2,1} = ...                               % guard
        [ y - yR;                       % y = yR
          yR - y;
          -ydot;                        % ydot < 0
          hX{2} ];                      % G \subset X                   % G \subset X
c{2,1} = (xvar(1) * polysin(xvar(3)) + l0 * sin(-al) - d_des) ^ 2;        % step length = l * sin(theta) + l0 * sin(-alpha)
h{2} = uvar^2;
H{2} = msspoly(0);


% Initial condition and Target Set
x0{2} = [ l0; 0; 0; 0.1 ];

% Target set is the entire space
hXT{1} = hX{1};
hXT{2} = hX{2};


[out] = HybridOCPDualSolver_switching(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,c,d,options);