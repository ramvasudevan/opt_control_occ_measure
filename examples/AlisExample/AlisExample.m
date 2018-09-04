clear;
T = 1;
d = 6;
nmodes = 2;
r2 = 0.3;

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

x0{2} = [ 0 ];

% Dynamics
x{1} = msspoly( 'x', 1 );
u{1} = msspoly( 'u', 1 );
f{1} = T * [ -x{1} ];
g{1} = T * [ 1 ];

x{2} = x{1};
u{2} = u{1};
f{2} = T * [ x{2} ];
g{2} = g{1};

% Domains
% Mode 1 
y = x{1};
hX{1} = y*(1-y);
hU{1} = 1 - u{1}.^2;
hXT{1} = [];

sX{1,2} = (y-1)^2;
R{1,2} = y;

h{1} = 0.5 * (y^2 + u{1}^2);
H{1} = 0;

% Mode 2
y = x{2};
hX{2} = (y-1)*(2-y);
hU{2} = 1 - u{2}.^2;
hXT{2} = hX{2};

h{2} = 0.5 * (y^2 + u{2}^2);
H{2} = 0.5 * y' * y;
% H{2} = y(1);

% Options
options.MinimumTime = 0;
options.withInputs = 1;
options.svd_eps = 1e4;

% Solve
dl0 = cell(nmodes, 1);
dl0{1} = boxMoments(x{1}, 0, 1);
% [out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);
[out] = HybridOCP_ValueFunc(t,x,u,f,g,hX,hU,sX,R,dl0,hXT,h,H,d,options)
pval = T * out.pval;

disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);