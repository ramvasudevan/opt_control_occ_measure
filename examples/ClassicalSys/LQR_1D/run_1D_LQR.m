% xdot = u
% x0 = 1;
% xT = [-0.5,0.5]
% 

clear;
T = 1;         % time horizon
d = 8;          % degree of relaxation
nmodes = 1;     % number of modes

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

x0{1} = 1;

% Dynamics
x{1} = msspoly( 'x', 1 );
u{1} = msspoly( 'u', 1 );
f{1} = T * [ 0 ];
g{1} = T * [ 1 ];

% Domains
% Mode 1 
y = x{1};
hX{1} = 2 - y'*y;
hU{1} = 1 - u{1}^2;
hXT{1} = 0.5^2 - y^2;
h{1} = y * y + 20 * u{1}^2;
H{1} = 0;

% Options
options.MinimumTime = 0;
options.withInputs = 0;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);

pval = T * out.pval;
disp(pval);
