function [out] = HybridOptimalControlDualSolver(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options)
% Assume running cost can be dependent on u

% Sanity check
if mod(d,2) ~= 0
    warning('d is not even. Using d+1 instead.');
    d = d+1;
end
nmodes = length(x);
m = length(u{1});
for i = 1 : nmodes
    if length(u{i}) ~= m
        error('Inconsistent matrix size.');
    end
    if (length(f{i}) ~= length(x{i})) || (size(g{i},1) ~= length(x{i}))
        error('Inconsistent matrix size.');
    end
    if size(g{i},2) ~= m
        error('Inconsistent matrix size.');
    end
end

svd_eps = 1e3;

if isfield(options, 'svd_eps'), svd_eps = options.svd_eps; end
if ~isfield(options, 'freeFinalTime'), options.freeFinalTime = 0; end
if ~isfield(options, 'withInputs'), options.withInputs = 0; end
if ~isfield(options, 'MinimumTime'), options.MinimumTime = 0; end
if options.MinimumTime == 1
    options.freeFinalTime = 1;
end

T = 1;
hT = t*(T-t);

if isempty(R)
    R = cell(nmodes,nmodes);
    for i = 1 : nmodes
    for j = 1 : nmodes
        if ~isempty(sX{i,j}), R{i,j} = x{j}; end
    end
    end
end

% define the program
prog = spotsosprog;
prog = prog.withIndeterminate( t );

% constraints to keep track of
mu_idx = zeros(nmodes, 1);
yp_idx = zeros(nmodes, size(g{1},2));
yn_idx = zeros(nmodes, size(g{1},2));

% Create program variables in each mode
for i = 1 : nmodes
    hU{ i } = 1 - u{ i }.^2;
    
    prog = prog.withIndeterminate( x{i} );
    prog = prog.withIndeterminate( u{i} );
    
    % create v(i)
    vmonom{ i } = monomials( [ t; x{ i } ], 0:d );
    [ prog, v{ i }, ~ ] = prog.newFreePoly( vmonom{ i } );
    
    % create w(i)
    wmonom{ i } = monomials( [ t; x{ i } ], 0:d );
    [ prog, w{ i }, ~ ] = prog.newFreePoly( wmonom{ i } );
    
    % create p(i,j)
    p{ i } = msspoly( zeros( length(u{i}), 1 ) );
    for j = 1 : length(u{i})
        pmonom{ i, j } = monomials( [ t; x{ i } ], 0:d );
        [ prog, p{ i }( j ), ~ ] = prog.newFreePoly( pmonom{ i, j } );
    end
    
    % create q(i)
    qmonom{ i } = monomials( [ t; x{ i } ], 0:d );
    [ prog, q{ i }, ~ ] = prog.newFreePoly( qmonom{ i } );
    
    % create the variables that will be used later
%     v0{ i } = subs( v{ i }, t, 0 );
    vT{ i } = subs( v{ i }, t, T );
    dvdt{ i } = diff( v{ i }, t );
    dvdx{ i } = diff( v{ i }, x{ i } );
    Lfv{ i } = dvdt{ i } + dvdx{ i } * f{ i };
    Lgv{ i } = dvdx{ i } * g{ i };
    Lv{ i } = Lfv{ i } + Lgv{ i } * u{ i };
    
    dwdt{ i } = diff( w{ i }, t );
    dwdx{ i } = diff( w{ i }, x{ i } );
    Lfw{ i } = dwdt{ i } + dwdx{ i } * f{ i };
    Lgw{ i } = dwdx{ i } * g{ i };
    Lw{ i } = Lfw{ i } + Lgw{ i } * u{ i };
end

% creating the constraints and cost function
obj = 0;
for i = 1 : nmodes
    % Lv + Lw + q*I_U + h_i >= 0        Dual: mu
    prog = sosOnK( prog, Lv{ i } + Lw{ i } + q{ i } + h{ i } , ...
                   [ t; x{ i }; u{ i } ], [ hT; hX{ i }; hU{ i } ], d );
    mu_idx(i) = size(prog.sosExpr, 1);
    
    % v(T,x) <= H_i(x)                  Dual: muT
    if ~isempty( hXT{ i } )
        if options.freeFinalTime
            prog = sosOnK( prog, H{ i } - v{ i }, [ t; x{ i } ], [ hT; hXT{ i } ], d );
        else
            prog = sosOnK( prog, H{ i } - vT{ i }, x{ i }, hXT{ i }, d );
        end
    end
    
    % v_i(t,x) <= v_j (t,R(x))          Dual: muS
    for j = 1 : nmodes
        if ( ~isempty( sX{ i, j } ) ) % if its empty there isn't a guard between these
            vj_helper = subs( v{ j }, x{ j }, R{ i, j } ); 
            prog = sosOnK( prog, vj_helper - v{ i }, [ t; x{ i } ] , [ hT; sX{ i, j } ], d );
        end
    end
    
    % Lfw + sum_j( p(i,j) ) + q <= 0    Dual: gamma
    prog = sosOnK( prog, -Lfw{ i } - sum( p{ i } ) - q{ i }, [ t; x{ i } ], [ hT; hX{ i } ], d );
    
    % | Lgw(i,j) | <= p(i,j), p(i,j) >= 0
    for j = 1 : size( g{i}, 2 )
        % sigma_P
        prog = sosOnK( prog, p{ i }( j ) - Lgw{ i }( j ), [ t; x{ i } ], [ hT; hX{ i } ], d );
        yp_idx(i,j) = size(prog.sosExpr, 1);
        % sigma_N
        prog = sosOnK( prog, p{ i }( j ) + Lgw{ i }( j ), [ t; x{ i } ], [ hT; hX{ i } ], d );
        yn_idx(i,j) = size(prog.sosExpr, 1);
        % sigma_H
        prog = sosOnK( prog, p{ i }( j ), [ t; x{ i } ], [ hT; hX{ i } ], d );
    end
    
    if ~isempty( x0{ i } )
        obj = obj + subs( v{ i }, [ t; x{i} ], [ 0; x0{i} ] );
    end
end

% set options
spot_options = spot_sdp_default_options();
spot_options.verbose = 1;

params.alpha = 1.5;
params.alpha = 1.5;
params.rho_x = 1e-3;
params.max_iters = 1e8;
params.eps = 1e-3;
params.verbose = 0;
params.normalize = 0;

spot_options.solver_options.scs = params;


%% Solve
tic;
[sol, y, dual_basis, ~] = prog.minimize( -obj, @spot_mosek, spot_options );
out.time = toc;

out.pval = double(sol.eval(obj));
out.sol = sol;

%% Extract control input
if ~options.withInputs
    out.u = [];
    return;
end

uout = cell(nmodes,size(g{1},2));
u_real_basis = cell( nmodes, 1 );
for i = 1 : nmodes
    urb = monomials( [ t; x{ i } ], 0:d / 2 );
    u_real_basis{ i } = urb;
    % moments of mu
    mu_basis = dual_basis{ mu_idx(i) };
    y_mu = sol.dualEval( y{ mu_idx(i) } );
    
    % moment matrix of mu
    mypoly = mss_s2v( urb * urb' );
    coeff_mu = DecompMatch( mypoly, mu_basis );
    moment_mat = double( mss_v2s( coeff_mu * y_mu ) );
    
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
    
    % yp and yn
    for j = 1 : size(g{i},2)
        % moments of sigma_p and sigma_n
        u_basis = dual_basis{ yp_idx( i, j ) };
        y_sp = sol.dualEval( y{ yp_idx( i, j ) } );
        y_sn = sol.dualEval( y{ yn_idx( i, j ) } );
        
        % rearrange
        coeff_sigma = DecompMatch( urb, u_basis );
        yp = coeff_sigma * y_sp;
        yn = coeff_sigma * y_sn;
        
        % solve for u
        u_coeff = iMyt * ( yp - yn );
        uout{ i, j } = u_coeff' * urb;
    end
end

out.u = uout;
