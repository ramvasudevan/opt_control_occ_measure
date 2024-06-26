% Double integrator with 2 modes - minimum time problem
% xdot = [ x_2 ] + [ 0 ] * u
%        [ 0   ]   [ 1 ]
% 
% u(t) \in [-1, 1]
% X_1 = { x | x_1^2 + x_2^2 <= r2 }
% X_2 = { x | x_1^2 + x_2^2 >= r2 }
% XT_1 = {(0,0)}
% XT_2 = {empty}
% h = 1, H = 0
% 
% Trajectory starts at (0.3,1) in mode 2, and arrives at (0,0) in minimum
% time.
% 


clear;
T = 5;
d = 6;
dr = 8;
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

x0{2} = [ 0.3; 1 ];

% Dynamics
x{1} = msspoly( 'x', 2 );
u{1} = msspoly( 'u', 1 );
f{1} = T * [ x{1}(2); 0 ];
g{1} = T * [ 0; 1 ];

x{2} = x{1};
u{2} = u{1};
f{2} = f{1};
g{2} = g{1};

% Domains and transitions
% Mode 1 
y = x{1};
hX{1} = r2 - y'*y;      % { x | x_1^2 + x_2^2 <= r2 }
hU{1} = 1 - u{1}^2;     % [-1, 1]
hXT{1} = 0-y.^2;        % {(0,0)}
h{1} = 1;
H{1} = 0;

% Mode 2
y = x{2};
hX{2} = y'*y - r2;      % { x | x_1^2 + x_2^2 >= r2 }
hU{2} = 1 - u{2}^2;     % [-1, 1]
sX{2,1} = [ r2 - y' * y;
            y' * y - r2 ];
R{2,1} = x{1};          % Identity reset map
h{2} = 1;
H{2} = 0;

% Options
options.freeFinalTime = 1;
options.withTransition = 1;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCPDualSolver_transition(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,dr,options);

pval = T * out.pval;

% Calculate actual T
xs0 = x0{2};
if xs0(1) >= -(xs0(2)^2-2)/2
    Ta = 1+xs0(1)+xs0(2)+xs0(2)^2/2;
elseif xs0(1) >= -xs0(2)^2/2*sign(xs0(2)) 
    Ta = 2*sqrt(xs0(1)+xs0(2)^2/2)+xs0(2);
else
    Ta = 2*sqrt(-xs0(1)+xs0(2)^2/2)-xs0(2);
end

disp(pval / Ta);
disp(['Minimum time = ' num2str(Ta)]);
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot

