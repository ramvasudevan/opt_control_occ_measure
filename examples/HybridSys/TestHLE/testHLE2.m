% Double integrator - Minimum Time

clear;
T = 2;
d = 6;
nmodes = 1;

% Define variables
t = msspoly( 't', 1 );
x = cell( nmodes, 1 );
u = cell( nmodes, 1 );
f = cell( nmodes, 1 );
g = cell( nmodes, 1 );
x0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
hU = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );

x0{1} = 0.5;

% Dynamics
x{1} = msspoly( 'x', 1 );
u{1} = msspoly( 'u', 1 );
f{1} = T * 1;
g{1} = 0;

% Domains
% Mode 1 
y = x{1};
hX{1} = 1 - y'*y;
hXT{1} = -(y+0.5)^2;
hU{1} = 1 - u{1}^2;
sX{1,1} = -(y-1)^2;
R{1,1} = -1;
h{1} = -1;
H{1} = 0;


% Options
options.freeFinalTime = 1;
options.withInputs = 0;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);

pval = T * out.pval;

