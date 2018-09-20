function [out] = SOCPDualSolver_scaled( t, y, u, fy, gy, sigma, dly0, hU, hy, Hy, degree, opts )
% 
% We assume a change of variables:      x = arctanh( y )
% or equivalently,                      y = tanh( x )
% As a result,
%     dy/dx = 1-y^2
%     ddy/dxdx = 2y(y^2-1)
% 

fprintf('Initializing...');

if mod(degree,2) ~= 0
    warning('d is not even. Using d+1 instead.');
    degree = degree+1;
end

svd_eps = 1e3;
% 
if isfield(opts, 'svd_eps'), svd_eps = opts.svd_eps; end
if ~isfield(opts, 'withInputs'), opts.withInputs = 0; end
if ~isfield(opts, 'MinimumTime'), opts.MinimumTime = 0; end
if opts.MinimumTime == 1
    opts.freeFinalTime = 1;
end

T = 1;
hT = t*(T-t);

% define the program
prog = spotsosprog;
prog = prog.withIndeterminate( t );
prog = prog.withIndeterminate( y );
prog = prog.withIndeterminate( u );

% Create variables
monom = monomials( [t;y], 0:degree );   % the basis of w

[ prog, w, ~ ] = prog.newFreePoly( monom );

% =======================================================================
% The following results are computed using chain rule:
%   ->  dv/dt = dw/dt
%   ->  dv/dx = dw/dy * dy/dx
%   ->  ddv/d(x_i)d(xj) = 
%           ddw/d(y_i)d(y_j) * d(y_i)/d(x_i) * d(y_j)/d(x_j) + ...
%                                   dw/d(y_j) * dd(y_i)/d(x_i)d(x_j)
% =======================================================================
dvdt = diff( w, t );
dvdx = diff( w, y ) .* (1 - y.^2);
Hess = diff( diff(w,y)', y ) .* ((1 - y.^2) * (1 - y.^2)' ) + diag( diff(w,y) .* ( 2 * y .* (y.^2 - 1) ) );

Av = dvdt + dvdx * ( fy + gy * u ) + 0.5 * sigma' * Hess * sigma;

vT = subs( w, t, T );
v0 = subs( w, t, 0 );

% Constraints
hY = 1 - y.^2;
% Av + h >= 0 : mu
prog = sosOnK( prog, Av + hy, [ t; y; u ], [ hT; hY; hU ], degree );
mu_idx = size(prog.sosExpr, 1);

% H - v(T,x) >= 0 : muT
prog = sosOnK( prog, Hy - vT, [ y ], [ hY ], degree );

% Objective function
obj = dly0( v0 );

% options
options = spot_sdp_default_options();
options.verbose = 1;
options.solveroptions = [];

fprintf('done\n');

%% Solve
fprintf('Solving...');
tic;
[ sol, dual_var, dual_basis ] = prog.minimize( -obj, @spot_mosek, options );
out.time = toc;
fprintf('done\n');

out.pval = double(sol.info.solverInfo.itr.pobjval);
out.sol = sol;
out.v = w;

%% Extract control input
if opts.withInputs == 0
    out.u_coeff = [];
    out.u = [];
    return;
end

% Basis
mu_basis = dual_basis{ mu_idx };
urb = monomials( [ t; y ], 0 : degree/2 );      % real basis of u

% Build moment matrix of mu
mypoly = mss_s2v( urb * urb' );
coeff_mu = DecompMatch( mypoly, mu_basis );
y_mu = sol.dualEval( dual_var{ mu_idx } );
moment_mat = double( mss_v2s( coeff_mu * y_mu ) );

% Pseudo-inverse of M_mu
[U,S,V] = svd(moment_mat);
iS1 = S;
startExp = 1e-10;
while norm(pinv(iS1)) / norm(S) > svd_eps
    iS1(iS1 < startExp) = 0;
    startExp = startExp * 10;
end
disp(fprintf('norm of moment matrix %0.2f\n', norm(S)))
disp(fprintf('norm of inverse moment matrix %0.2f\n', norm(pinv(iS1))))

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