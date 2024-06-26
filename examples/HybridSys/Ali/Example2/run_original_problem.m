% Counterexample generated by Ali
% 

T = 1;         % time horizon
d = 6;          % degree of relaxation
nmodes = 2;     % number of modes

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
x0 = cell( nmodes, 1 );


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
hX{1} = (y-1)*(2-y);      % X1 = [1,2]
hU{1} = 4 - u{1}^2;
hXT{1} = [];
h{1} = 0.5 * (y^2 + u{1}^2 );
H{1} = 0;

sX{1,2} = [ 1 - y
            y - 1 ];
R{1,2} = y;

x0{1} = 1.5;

% dl0{ 1 } = sphericalMoments( x{1}, [0;0], sqrt(r2) );
dl0{ 1 } = [];

% Mode 2
y = x{2};
hX{2} = y*(1-y);        % X2 = [0,1]
hU{2} = 4 - u{2}^2;
hXT{2} = hX{2};

h{2} = 0.5 * (y^2 + u{2}^2 );
H{2} = 0.5 * y * y;

% dl_sphere = sphericalMoments( x{2}, [0;0], sqrt(r2) );
% dl_box = boxMoments( x{2}, [-1;-1], [1;1] );
% dl0{ 2 } = @( p ) ( dl_box(p) - dl_sphere(p) );

% Options
options.MinimumTime = 0;
options.withInputs = 1;
options.svd_eps = 1e4;

% Optimal control provided by Hamiltonian condition
u_coeff = cell( nmodes, 1 );
u_coeff{ 1 } = 1;
u_coeff{ 2 } = 1;

% Solve
% [out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);

dl0{1} = boxMoments( x{1}, 1, 1.5 );
% [out] = HybridOCP_ValueFunc(t,x,u,f,g,hX,hU,sX,R,dl0,hXT,h,H,d,options);
[out] = HybridOCP_ValueFunc_Hamiltonian(t,x,u,f,g,hX,u_coeff,sX,R,dl0,hXT,h,H,d,options);

pval = T * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%%
v1 = out.sol.eval( out.v{1} );
v2 = out.sol.eval( out.v{2} );

% v1  = diff(v1, t);
% v2 = diff(v2, t);

Hamiltonian2 = diff( v2, t ) + diff( v2, x{2} ) * subs( f{2}+g{2}*u{2}, u{2}, out.u{2} ) + subs( h{2}, u{2}, out.u{2} );

% v1_T = subs( v1, t, 1 );
% v2_0 = subs( v2, t, 0 );
% [ xval, yval ] = meshgrid( -1:0.01:1, -1:0.01:1 );
% v1_val = reshape( double( msubs( v2_0, x{1}, [ xval(:), yval(:) ]' ) ), 201, 201 );
% mesh( xval, yval, v1_val );
figure(1);
hold on;
% hold on;

    [ xval, yval ] = meshgrid( 0:0.01:1, 1 : 0.01 : 1.5 );
    zval = reshape( double( msubs( v1, [t;x{1}], [ xval(:), yval(:) ]' ) ), 51, 101 );
    surf( xval, yval, zval );
    xlabel('t');
    ylabel('x');

    [ xval2, yval2 ] = meshgrid( 0:0.01:1, 0 : 0.01 : 1 );
    zval2 = reshape( double( msubs( v2, [t;x{2}], [ xval2(:), yval2(:) ]' ) ), 101, 101 );
    surf( xval2, yval2, zval2 );

% hold on;
% zval = xval.^2 + yval.^2;
% mesh(xval, yval, zval);