function [out] = OCPDualSolver( t, x, u, f, g, x0, hX, hXT, h, H, degree, opts )
% Running cost CAN be dependent on u
% not using sigma idea, so it should run faster.
% 
% xdot = F(t,x,u) = f(t,x) + g(t,x)u
% 
% Dual problem:
% 
% sup  v( 0, x0 )
% s.t. LFv + h >= 0,      on [0,T] x X x U,     dual: mu
%      H - v(T,x) >= 0,   on {T} x XT,      	dual: muT
% 
% ATTENTION: LF is different from Lf !!!
% 

fprintf('Initializing...')

if mod(degree,2) ~= 0
    warning('d is not even. Using d+1 instead.');
    degree = degree+1;
end

svd_eps = 1e3;

if isfield(opts, 'svd_eps'), svd_eps = opts.svd_eps; end
if ~isfield(opts, 'freeFinalTime'), opts.freeFinalTime = 0; end
if ~isfield(opts, 'withInputs'), opts.withInputs = 0; end
if ~isfield(opts, 'MinimumTime'), opts.MinimumTime = 0; end
if opts.MinimumTime == 1
    opts.freeFinalTime = 1;
end

T = 1;
hT = t*(T-t);
hU = 1 - u.^2;
m = length(u);

% Define the program
prog = spotsosprog;
prog = prog.withIndeterminate( t );
prog = prog.withIndeterminate( x );
prog = prog.withIndeterminate( u );

% Create variables
monom = monomials( [t;x], 0:degree );   % the basis of v

[ prog, v, ~ ] = prog.newFreePoly( monom );

Lfv = diff(v,t) + diff(v,x) * f;
Lgv = diff(v,x) * g;
LFv = Lfv + Lgv * u;
vT = subs( v, t, T );
v0 = subs( v, [t;x], [0;x0] );

% Constraints

% LFv + h >= 0 : mu
prog = sosOnK( prog, LFv + h, [ t; x; u ], [ hT; hX; hU ], degree );
mu_idx = size(prog.sosExpr, 1);

% H - v(T,x) >= 0 : muT
if opts.freeFinalTime == 1
    prog = sosOnK( prog, H - v, [ t; x ], [ hT; hXT ], degree );
else
    prog = sosOnK( prog, H - vT, [ x ], [ hXT ], degree );
end

% Objective function
obj = v0;

% options
options = spot_sdp_default_options();
options.verbose = 1;
options.solveroptions = [];

fprintf('done\n');

%% Solve
fprintf('Solving...');
tic;
[ sol, y, dual_basis, ~ ] = prog.minimize( -obj, @spot_mosek, options );
out.time = toc;
fprintf('done\n');

out.pval = double(sol.eval(v0));
out.sol = sol;

%% Extract control input
if opts.withInputs == 0
    out.u_coeff = [];
    out.u = [];
    return;
end

% Basis
mu_basis = dual_basis{ mu_idx };
urb = monomials( [ t; x ], 0 : degree/2 );      % real basis of u

% Build moment matrix of mu
mypoly = mss_s2v( urb * urb' );
coeff_mu = DecompMatch( mypoly, mu_basis );
y_mu = sol.dualEval( y{ mu_idx } );
moment_mat = double( mss_v2s( coeff_mu * y_mu ) );

% Pseudo-inverse of M_mu
[U,S,V] = svd(moment_mat);
iS1 = S;
startExp = 1e-10;
while norm(pinv(iS1)) / norm(S) > svd_eps
    iS1(iS1 < startExp) = 0;
    startExp = startExp * 10;
end
disp(sprintf('norm of moment matrix %0.2f', norm(S)))
disp(sprintf('norm of inverse moment matrix %0.2f', norm(pinv(iS1))))

iMyt = V*pinv(iS1)*U';

% Solve for u
for j = 1 : m
    mypoly = u(j) * urb;
    coeff_umu = DecompMatch( mypoly, mu_basis );
    y_u = sol.dualEval( coeff_umu * y_mu );
    
    u_coeff = iMyt * y_u;
    out.u{j} = u_coeff' * urb;
    out.u_coeff{j} = u_coeff;
end
