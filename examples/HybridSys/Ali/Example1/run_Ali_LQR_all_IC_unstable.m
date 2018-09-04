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
% Trajectory starts at (1,1) in mode 2, and minimizes the running cost
% h = x'*x + 20 * u^2 up to time T
% 

% clear;
% close all;

T = 5;         % time horizon
d = 6;          % degree of relaxation
nmodes = 2;     % number of modes
r2 = 0.25;       % r^2, where r is the radius of the domain of mode 1

% Define variables
t = msspoly( 't', 1 );
x = cell( nmodes, 1 );
u = cell( nmodes, 1 );
f = cell( nmodes, 1 );
g = cell( nmodes, 1 );
dl0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
hU = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );

x0{2} = [ 2; 2 ];

% Dynamics
x{1} = msspoly( 'x', 2 );
u{1} = msspoly( 'u', 1 );
f{1} = T * [ x{1}(2); -x{1}(1) - 2*x{1}(2) ];
g{1} = T * [ 0; 10 ];

x{2} = x{1};
u{2} = u{1};
f{2} = T * [ x{2}(2); -x{2}(1) + 2*x{2}(2) ];
g{2} = g{1};

% Domains
% Mode 1 
y = x{1};
hX{1} = r2 - y'*y;
hU{1} = 1 - u{1}^2;
hXT{1} = hX{1};
h{1} = 0.5 * (10*u{1})^2;
H{1} = y' * y;

% dl0{ 1 } = sphericalMoments( x{1}, [0;0], sqrt(r2) );
dl0{ 1 } = [];

% Mode 2
y = x{2};
hX{2} = [ 1 - x{2}.^2;
          y'*y - r2 ];
hU{2} = 1 - u{2}^2;
hXT{2} = hX{2};
sX{2,1} = [ r2 - y' * y;
            y' * y - r2 ];
R{2,1} = x{1};
h{2} = 0.5 * (10*u{2})^2;
H{2} = y' * y;

dl_sphere = sphericalMoments( x{2}, [0;0], sqrt(r2) );
dl_box = boxMoments( x{2}, [-1;-1], [1;1] );
dl0{ 2 } = @( p ) ( dl_box(p) - dl_sphere(p) );

% Options
options.MinimumTime = 0;
options.withInputs = 1;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCP_ValueFunc(t,x,u,f,g,hX,hU,sX,R,dl0,hXT,h,H,d,options);

pval = T * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%%
v1 = out.sol.eval( out.v{1} );
v2 = out.sol.eval( out.v{2} );

Hamiltonian2 = diff( v2, t ) + diff( v2, x{2} ) * subs( f{2}+g{2}*u{2}, u{2}, out.u{2} ) + subs( h{2}, u{2}, out.u{2} );

% v1_T = subs( v1, t, 1 );
% v2_0 = subs( v2, t, 0 );
% [ xval, yval ] = meshgrid( -1:0.01:1, -1:0.01:1 );
% v1_val = reshape( double( msubs( v2_0, x{1}, [ xval(:), yval(:) ]' ) ), 201, 201 );
% mesh( xval, yval, v1_val );
figure(1);
% hold on;
for i = 0 : 100
    v2_0 = T * subs( v2, t, i/100 );
    Ht = subs( Hamiltonian2, t, i/100 );
    [ xval, yval ] = meshgrid( -1:0.01:1, -1:0.01:1 );
    zval = reshape( double( msubs( v2_0, x{1}, [ xval(:), yval(:) ]' ) ), 201, 201 );
%     zval = reshape( double( msubs( Ht, x{2}, [xval(:), yval(:)]' ) ), 201, 201 );
    mesh( xval, yval, zval );
    pause;
end

% hold on;
% zval = xval.^2 + yval.^2;
% mesh(xval, yval, zval);