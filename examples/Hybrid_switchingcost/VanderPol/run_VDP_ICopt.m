function [basis, moments] = run_VDP_ICopt( offset )
% offset = -0.4;

T = 6;
d = 10;
nmodes = 2;

% Define variables
t = msspoly( 't', 1 );
x = cell( nmodes, 1 );
u = cell( nmodes, 1 );
f = cell( nmodes, 1 );
g = cell( nmodes, 1 );
hX0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
hU = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );
c = cell( nmodes, 1 );

var = msspoly( 'x', 2 );
x{1} = var;
x{2} = var;

% Dynamics
f{1} = T * VDP_Dyn( var );
f{2} = f{1};

% Domains/guards/reset maps
% Mode 1 : [-1,0]x[-1,1]
hX{1} = [ -var(1) * (1 + var(1))
          1 - var(2)^2 ];
sX{1,2} = [ var(1)              % x1 == 0
            -var(1)
            var(2)
            1 - var(2)];           % x2 in [0,1]
R{1,2} = var;
c{1,2} = 0;

% Mode 2 : [0,1]x[-1,1]
hX{2} = [ var(1) * (1 - var(1))
          1 - var(2)^2 ];
sX{2,1} = [ var(1)              % x1 == 0
            -var(1)
            - var(2)
            var(2) + 1];         % x2 in [-1,0]
R{2,1} = var;
c{2,1} = 10*(var(2)-offset)^2;


% Cost functions
h{1} = msspoly(0);
h{2} = msspoly(0);

H{1} = msspoly(0);
H{2} = msspoly(0);          % (x1 - 0.5)^2

% Initial condition and target set
hXT{1} = hX{1};
hXT{2} = hX{2};

domain0 = [ -1, -0.05;
            0, 0 ];
hX0{1} = [ domain0(:,2) - var
           var - domain0(:,1) ];

% Solve
[out] = HybridOCPDualSolver_IC_noInput(t,x,u,f,g,hX,hU,sX,R,hX0,hXT,h,H,c,d,[]);

basis = out.mu0_basis{1};
moments = out.mu0_moments{1};