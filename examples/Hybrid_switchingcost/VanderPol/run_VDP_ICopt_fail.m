
T = 20;
d = 8;
nmodes = 3;

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
x{3} = msspoly('p',1);

s_helper = (1 - var(1)^2)*( 1 - var(2)^2 );

% Dynamics
f{1} = -T * VDP_Dyn( var );
f{2} = f{1};
f{3} = msspoly(0);

% Domains/guards/reset maps
% Mode 1 : [-1,0]x[-1,1]
hX{1} = [ -var(1) * (1 + var(1))
          1 - var(2)^2 ];
sX{1,2} = [ var(1)              % x1 == 0
            -var(1)
            var(2)-0.05 ];           % x2 in [0,1]
R{1,2} = var;
c{1,2} = - var(2)^2;

sX{1,3} = [ hX{1}
            s_helper
            -s_helper ];
R{1,3} = var(1);
c{1,3} = 100;

% Mode 2 : [0,1]x[-1,1]
hX{2} = [ var(1) * (1 - var(1))
          1 - var(2)^2 ];
sX{2,1} = [ var(1)              % x1 == 0
            -var(1)
            - var(2)-0.05 ];         % x2 in [-1,0]
R{2,1} = var;
c{2,1} = -var(2)^2;

sX{2,3} = [ hX{2}
            s_helper
            -s_helper ];
R{2,3} = var(1);
c{2,3} =100;

% Mode 3 : 
hX{3} = 2 - x{3}^2;


% Cost functions
h{1} = msspoly(0);
h{2} = msspoly(0);
h{3} = msspoly(0);

H{1} = msspoly(0);
H{2} = msspoly(0);          % (x1 - 0.5)^2
H{3} = msspoly(0);

% Initial condition and target set
hXT{1} = [hX{1};0.25 - var'*var];
hXT{2} = [hX{2};0.25 - var'*var];
hXT{3} = hX{3};

domain0 = [ -0.3, -0.3;
            0.1, 0.9 ];
hX0{1} = [ domain0(:,2) - var
           var - domain0(:,1) ];

% Solve
[out] = HybridOCPDualSolver_IC_noInput(t,x,u,f,g,hX,hU,sX,R,hX0,hXT,h,H,c,d,[]);