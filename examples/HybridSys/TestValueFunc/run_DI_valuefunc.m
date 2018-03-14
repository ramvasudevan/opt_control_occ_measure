% Double integrator with 2 modes
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
% Trajectory starts at (1,1) in mode 2, and minimizes the running cost
% h = x'*x + 20 * u^2 up to time T
% 
% Value function approximation
% 

clear;
T = 15;         % time horizon
d = 8;          % degree of relaxation
nmodes = 2;     % number of modes
r2 = 0.3;       % r^2, where r is the radius of the domain of mode 1

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

x0{2} = [ 1; 1 ];

% Dynamics
x{1} = msspoly( 'x', 2 );
u{1} = msspoly( 'u', 1 );
f{1} = T * [ x{1}(2); 0 ];
g{1} = T * [ 0; 1 ];

x{2} = x{1};
u{2} = u{1};
f{2} = f{1};
g{2} = g{1};

% Domains
% Mode 1 
y = x{1};
hX{1} = r2 - y'*y;
hU{1} = 1 - u{1}^2;
hXT{1} = hX{1};
h{1} = 1*x{1}' * x{1} + 20 * u{1}^2;
H{1} = 0;

% Mode 2
y = x{2};
hX{2} = [ 4 - y.^2;
          y'*y - r2 ];
hU{2} = 1 - u{2}^2;
hXT{2} = hX{2};
sX{2,1} = [ r2 - y' * y;
            y' * y - r2 ];
R{2,1} = x{1};
h{2} = 20*x{2}' * x{2} + 1 * u{2}^2;
H{2} = 0;

dl_sphere = sphericalMoments( x{2}, [0;0], sqrt(r2) );
dl_box = boxMoments( x{2}, [-1;-1], [1;1] );
dl0{ 2 } = @( p ) ( dl_box(p) - dl_sphere(p) );

% Options
options.MinimumTime = 0;
options.withInputs = 0;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCP_ValueFunc(t,x,u,f,g,hX,hU,sX,R,dl0,hXT,h,H,d,options);
% [out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);

pval = T * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
v1 = T * out.sol.eval( out.v{1} );
v2 = T * out.sol.eval( out.v{2} );

dv1dt = diff( v1, t );
dv2dt = diff( v2, t );
[ xval, yval ] = meshgrid( -1:0.01:1, -1:0.01:1 );

for i = 0 : 100
    tmp1 = subs( dv1dt, t, i/100 );
    tmp2 = subs( dv2dt, t, i/100 );
    val1 = reshape( double( msubs( tmp1, x{1}, [ xval(:), yval(:) ]' ) ), 201, 201 );
    val2 = reshape( double( msubs( tmp2, x{2}, [ xval(:), yval(:) ]' ) ), 201, 201 );
    
    val1( (xval.^2 + yval.^2) > r2 ) = nan;
    val2( (xval.^2 + yval.^2) < r2 ) = nan;
    
    figure(1);
    clf;
    hold on;
    surf( xval, yval, val1 );
    surf( xval, yval, val2 );
    view(3);
    title(['Time = ',num2str(i/100*T)]);
    pause;
end
