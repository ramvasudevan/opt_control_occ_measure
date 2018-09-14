clear;
T = 1;         % time horizon
d = 10;          % degree of relaxation

t = msspoly( 't', 1 );
y = msspoly( 'y', 1 );
u = msspoly( 'u', 1 );

% f =  -1 * x;
% g = 1;
fy = -1 * (y + y^3/3 + y^5/5);       % f = -x
gy = 1;
sigma = 0.3;

% x0 = 0.3;      % initial point
dly0 = boxMoments( y, -0.5, 0.5 );       % uniform distribution on [-1, 1]
% hX = 1 - x.^2;
% hXT = 1 - x.^2;
hU = 1 - u.^2;
% h = 0.5 * u^2;
% H = 0.5 * x.^2;
hy = 0.5 * u^2;
Hy = 0.5 * ( y^2 + 2 * y^4/3 + 23 * y^6/45 );
% Hy = 0.5 * ( y^2 + 2 * y^4/3 + 23 * y^6/45 + 44 * x^8/105 );

options.MinimumTime = 0;
options.freeFinalTime = 0;
options.withInputs = 0;

% [out] = SOCPDualSolver_noinput(t,x,u,f,sigma,x0,hX,hXT,h,H,d,options);
% [out] = SOCPDualSolver_ValueFunc( t, x, u, f, g, sigma, dl0, hX, hXT, hU, h, H, d, options );
[out] = SOCPDualSolver_scaled_dim1( t, y, u, fy, gy, sigma, dly0, hU, hy, Hy, d, options );

pval = T * out.pval;
disp(pval);

v = out.sol.eval(out.v);

PlotLQGsol( 30, t, x, out.u{1}, v );