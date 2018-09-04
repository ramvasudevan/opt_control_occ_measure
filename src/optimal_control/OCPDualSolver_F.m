function [out] = OCPDualSolver_F( t, x, u, F, x0, hX, hXT, hU, h, H, degree, opts )
% Running cost CAN be dependent on u
% System may not be affine in control
% 
% xdot = F(t,x,u)
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

if ~isfield(opts, 'freeFinalTime'), opts.freeFinalTime = 0; end
if ~isfield(opts, 'withInputs'), opts.withInputs = 0; end
if ~isfield(opts, 'MinimumTime'), opts.MinimumTime = 0; end
if opts.MinimumTime == 1
    opts.freeFinalTime = 1;
end

T = 1;
hT = t*(T-t);
% hU = 1 - u.^2;
m = length(u);

% Define the program
prog = spotsosprog;
prog = prog.withIndeterminate( t );
prog = prog.withIndeterminate( x );
prog = prog.withIndeterminate( u );

% Create variables
monom = monomials( [t;x], 0:degree );   % the basis of v

[ prog, v, ~ ] = prog.newFreePoly( monom );

LFv = diff(v,t) + diff(v,x) * F;
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
[ sol, y, dual_basis ] = prog.minimize( -obj, @spot_mosek, options );
out.time = toc;
fprintf('done\n');

out.pval = double(sol.eval(v0));
out.sol = sol;
