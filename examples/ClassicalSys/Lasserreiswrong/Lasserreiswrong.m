% Compare Lasserre's and Vinter's result on a system that's not affine in
% control

clear;
clc;
d = 10;

t = msspoly('t', 1);
x = msspoly('x', 1);
u = msspoly('u', 1);

x0 = msspoly( 0 );
opts = [];

%% Lasserre
% Dynamics
F = ( u - 1 ) ^ 2;
h = 10 * x + u;
H = msspoly( 0 );

hX = x * ( 1 - x );
hXT = hX;
hU = u * ( 1 - u );

[sol] = OCPDualSolver_F( t, x, u, F, x0, hX, hXT, hU, h, H, d, opts );

fprintf( 'Pval = %f\n', sol.pval )
pause;

%% Vinter
F = u;
h = 10 * x + 0.105573 - 0.559017*(u-0.8) + 0.174693*(u-0.8)^2 - 0.109183*(u-0.8)^3 + 0.0852992*(x-0.8)^4 - 0.0746368*(x-0.8)^5;
H = msspoly( 0 );

hX = x * ( 1 - x );
hXT = hX;
hU = u * ( 1 - u );

[sol] = OCPDualSolver_F( t, x, u, F, x0, hX, hXT, hU, h, H, d, opts );

fprintf( 'Pval = %f\n', sol.pval )