clear;
T = 1;         % time horizon
d = 8;          % degree of relaxation

t = msspoly( 't', 1 );
x = msspoly( 'x', 1 );
u = msspoly( 'u', 1 );

f = -x;
g = 1;
sigma = 0.1;

x0 = 0.3;      % initial point
hX = 1 - x.^2;
hXT = 1 - x.^2;
hU = 1 - u.^2;
h = 0.5 * u^2;
H = 0.5 * x.^2;

options.MinimumTime = 0;
options.freeFinalTime = 0;

% [out] = SOCPDualSolver_noinput(t,x,u,f,sigma,x0,hX,hXT,h,H,d,options);
[out] = SOCPDualSolver( t, x, u, f, g, sigma, x0, hX, hXT, hU, h, H, d, options );

pval = T * out.pval;
disp(pval);

out.sol.eval(out.v)