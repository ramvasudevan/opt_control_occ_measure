function [ out ] = OCP_DualSolver( t, x, u, f, x0, hX, hXT, hU, T, h, H, degree, opts )

% xdot = f(t,x,u)
% 
% sup  v(0,x0)
% s.t. Lfv + h(t,x,u) >=0, on [0,T] x X x U
%      v(x) <= H(x),       on XT
%    (or: v(t,x) <= H(x), on [0,T] x XT, for free terminal time)
%      

fprintf('Initializing...')

scaling = T;
T = 1;
hT = t*(T-t);
f = f * scaling;

% Define the program
prog = spotsosprog;
prog = prog.withIndeterminate( t );
prog = prog.withIndeterminate( x );
prog = prog.withIndeterminate( u );

% Create program variables
vmonom = monomials( [ t; x ], 0:degree );

[ prog, v, ~ ] = prog.newFreePoly( vmonom );
Lfv = diff(v,t) + diff(v,x) * f;
v0 = subs( v, [ t; x ], [ 0; x0 ] );
vT = subs( v, t, T );

% Constraints
prog = sosOnK( prog, h + Lfv, [ t; x; u ], [ hT; hX; hU ], degree );

if opts.freeT == 1
    prog = sosOnK( prog, H - v, [ t; x ], [ hT; hXT ], degree );
else
    prog = sosOnK( prog, H - vT, [ x ], [ hXT ], degree );
end

% Objective function
obj = -v0;

% options
options = spot_sdp_default_options();
options.verbose = 1;
% options.scale_monomials = opts.scale_monomials;
% options.domain_size = domain_size;
options.solveroptions = [];

fprintf('done\n');

%% Solve
fprintf('Solving...');
[ sol ] = prog.minimize( obj, @spot_mosek, options );
fprintf('done\n');

out.sol = sol;
out.v = sol.eval(v);
out.pval = double(sol.eval(v0))*scaling;
