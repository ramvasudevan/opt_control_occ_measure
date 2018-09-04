clear;
T = 1;         % time horizon
d = 8;          % degree of relaxation

t = msspoly( 't', 1 );
x = msspoly( 'x', 2 );
u = msspoly( 'u', 1 );

A = [ 0, 1; 0, 0 ];
B = [ 0; 1 ];
sigma = [ 1; 1 ];
f = A * x + B * u;

x0 = [ 0.5; 0.5 ];      % initial point
hX = 1 - x.^2;
hXT = 1 - x.^2;
h = 10 * (x' * x) + u^2;
H = msspoly( 0 );

options.MinimumTime = 0;
options.freeFinalTime = 0;

[out] = SOCPDualSolver_noinput(t,x,u,f,sigma,x0,hX,hXT,h,H,d,options);

pval = T * out.pval;
disp(pval);