function [out] = OCP_Controller_Dual( t, x, u, f, g, x0, hX, hXT, h, H, degree, opts )
% Assuming running cost is dependent on u

% 
% xdot = F(t,x,u) = f(t,x) + g(t,x)u
% 
% Dual problem:
% 
% sup  v( 0, x0 )
% s.t. LFv + LFw + q + h >= 0,      on [0,T] x X x U,   mu
%      H - v(T,x) >= 0,             on {T} x XT,      	muT
%      -Lfw - sum(p_j) - q >= 0     on [0,T] x X,       gamma
%      p - Lgw >=0, p+Lgw >= 0,     on [0,T] x X,       sigmaPlus, sigmaMinus
%      p >= 0,                      on [0,T] x X,       sigmaHat
% 
% ATTENTION: LF is different from Lf !!!
% 

fprintf('Initializing...')

if mod(degree,2) ~= 0
    warning('d is not even. Using d+1 instead.');
    degree = degree+1;
end

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
monom = monomials( [t;x], 0:degree );   % v, w, p, q share the same basis
p = msspoly(zeros(m,1));

[ prog, v, ~ ] = prog.newFreePoly( monom );
[ prog, w, ~ ] = prog.newFreePoly( monom );
for j = 1 : m
    [ prog, p(j), ~ ] = prog.newFreePoly( monom );
end
[ prog, q, ~ ] = prog.newFreePoly( monom );

Lfv = diff(v,t) + diff(v,x) * f;
Lgv = diff(v,x) * g;
LFv = Lfv + Lgv * u;
Lfw = diff(w,t) + diff(w,x) * f;
Lgw = diff(w,x) * g;
LFw = Lfw + Lgw * u;
vT = subs( v, t, T );
v0 = subs( v, [t;x], [0;x0] );

% Constraints
sp_idx = zeros(j,1);
sn_idx = zeros(j,1);

% LFv + LFw + q + h >= 0 : mu
prog = sosOnK( prog, LFv + LFw + q + h, [ t; x; u ], [ hT; hX; hU ], degree );
% H - v(T,x) >= 0 : muT
if opts.freeFinalTime == 1
    prog = sosOnK( prog, H - v, [ t; x ], [ hT; hXT ], degree );
else
    prog = sosOnK( prog, H - vT, [ x ], [ hXT ], degree );
end
% -Lfw - sum(p_j) - q >= 0 : gamma
prog = sosOnK( prog, -Lfw - sum(p) - q, [ t; x ], [ hT; hX ], degree );
mu_idx = size(prog.sosExpr, 1);
% p - Lgw >=0, p+Lgw >= 0 : simgaP, sigmaM
% p >= 0 : sigmaHat
for j = 1 : m
    prog = sosOnK( prog, p(j) - Lgw(j), [ t; x ], [ hT; hX ], degree );
    sp_idx(j) = size(prog.sosExpr, 1);

    prog = sosOnK( prog, p(j) + Lgw(j), [ t; x ], [ hT; hX ], degree );
    sn_idx(j) = size(prog.sosExpr, 1);
    
    prog = sosOnK( prog, p(j), [ t; x ], [ hT; hX ], degree );
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
[ sol, y, dual_basis, ~ ] = prog.minimize( -obj, @spot_mosek, options );
fprintf('done\n');

out.pval = double(sol.eval(v0));
out.sol = sol;

%% Extract control input
if opts.withInputs == 0
    out.u_coeff = [];
    out.u = [];
    return;
end


moment_vec_yp = cell(m,1);
moment_vec_yn = cell(m,1);

% Basis
mu_basis = dual_basis{ mu_idx };
u_basis = dual_basis{ sp_idx(1) };
phi = monomials( [t; x], 0:degree/2 );

% Build moment matrix of mu
cnt = 1;
len = length(phi);
P = msspoly( zeros( len*(len+1)/2, 1 ) );
for a = 1 : length(phi)
    for b = 1 : a
        P(cnt) = phi(a) * phi(b);
        cnt = cnt + 1;
    end
end
if (cnt-1 ~= length(P))
    error('cnt != len');
end

moment_mu = sol.dualEval( y{mu_idx} );
coeff = DecompMatch( P, mu_basis );
moment_mat_mu = mss_v2s( double(coeff * moment_mu)' );

% Extract moments of sigma
coeff = DecompMatch( phi, u_basis );
for j = 1 : m
    moment_vec_yp{j} = coeff * sol.dualEval( y{sp_idx(j)} );
    moment_vec_yn{j} = coeff * sol.dualEval( y{sn_idx(j)} );
end

% Extract input
[U,S,V] = svd(moment_mat_mu);
iS1 = S;

startExp = 1e-10;
while norm(pinv(iS1)) / norm(S) > 100
    iS1(iS1 < startExp) = 0;
    startExp = startExp * 10;
end

disp(sprintf('norm of moment matrix %0.2f', norm(S)))
disp(sprintf('norm of inverse moment matrix %0.2f', norm(pinv(iS1))))

iMyt = V*pinv(iS1)*U';

out.u_coeff = cell(m,1);
out.u = cell(m,1);
for j = 1 : m
    u_coeff = (iMyt * (moment_vec_yp{j} - moment_vec_yn{j}));
    out.u_coeff{j} = u_coeff;
    out.u{j} = u_coeff' * phi;
end


