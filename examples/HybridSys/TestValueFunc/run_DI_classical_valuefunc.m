% Double integrator with 2 modes
% xdot = [ x_2 ] + [ 0 ] * u
%        [ 0   ]   [ 1 ]
% 
% u(t) \in [-1, 1]

clear;
T = 15;         % time horizon
d = 6;          % degree of relaxation
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

% x0{2} = [ 1; 1 ];  Not using this

% Dynamics
x{1} = msspoly( 'x', 2 );
u{1} = msspoly( 'u', 1 );
f{1} = T * [ x{1}(2); 0 ];
g{1} = T * [ 0; 1 ];


% Domains
% Mode 1 
y = x{1};
hX{1} = 1 - y.^2;
hU{1} = 1 - u{1}^2;
hXT{1} = hX{1};
h{1} = 1*x{1}' * x{1} + 20 * u{1}^2;
H{1} = 0;

dl_box = boxMoments( x{1}, [-1;-1], [1;1] );
dl0{ 1 } = dl_box;

% Options
options.MinimumTime = 0;
options.withInputs = 0;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCP_ValueFunc(t,x,u,f,g,hX,hU,sX,R,dl0,hXT,h,H,d,options);

pval = T * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);


% Plot
v1 = T * out.sol.eval( out.v{1} );

[ xval, yval ] = meshgrid( -1:0.01:1, -1:0.01:1 );

for i = 0 : 100                         % plot value function at time = 0:100
    tmp1 = subs( v1, t, i/100 );
    val1 = reshape( double( msubs( tmp1, x{1}, [ xval(:), yval(:) ]' ) ), 201, 201 );
    
    figure(1);
    clf;
    hold on;
    surf( xval, yval, val1 );
    view(3);
    title(['Time = ',num2str(i/100*T)]);
    pause;
end