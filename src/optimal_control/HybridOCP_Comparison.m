function [out] = HybridOCP_Comparison(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,seq,d,options)
% (Incorretly) solves hybrid optimal control problem
% * Do not optimize over the sequence of transition
% * Treat guards as target sets in each seperate mode
% ------------------------------------------------------------------------
% t     -- time indeterminate,      1-by-1 free msspoly
% x     -- state indeterminate,     I-by-1 cell of (n_i)-by-1 free msspoly
% u     -- control indeterminate,   I-by-1 cell of (m_i)-by-1 free msspoly
% f     -- dynamics, part 1,        I-by-1 cell of (n_i)-by-1 msspoly in x{i}
% g     -- dynamics, part 2,        I-by-1 cell of (n_i)-by-(m_i) msspoly in x{i}
% hX    -- domain (X_i),            I-by-1 cell of (~)-by-1 msspoly in x{i}
% hU    -- set U_i,                 I-by-1 cell of (~)-by-1 msspoly in u{i}
% sX    -- guard(i,j),              I-by-I cell of (~)-by-1 msspoly in x{i}
% R     -- reset map(i,j),          I-by-I cell of (n_j)-by-1 msspoly in x{i}
% x0    -- initial point,           I-by-1 cell of (n_i)-by-1 reals
% hXT   -- target set,              I-by-1 cell of (~)-by-1 msspoly in x{i}
% h     -- running cost,            I-by-1 cell of 1-by-1 msspoly in (t,x{i},u{i})
% H     -- terminal cost,           I-by-1 cell of 1-by-1 msspoly in x{i}
% seq   -- sequence of transition   1-by-(?) array of positive integers
% d     -- degree of relaxation,    positive even number (scalar)
% options  -- struct
% ------------------------------------------------------------------------
% Solves the following hybrid optimal control problem:
% 
% inf  { \int_0^1 h(t,x,u) dt } + H( x(1) )
% s.t. \dot{x_}
% s.t. \dot{x_i} = f_i + g_i * u
%      x_j(t+) = R_ij( x_i(t-) ) when { x_i(t-) \in S_ij }
%      x(0) = x0
%      x(t) \in X
%      x(T) \in XT
%      u(t) \in U
% 
% where:
% X_i  = { x | each hX{i}(x) >= 0 }          domain of mode i
% U_i  = { u | each hU{i}(u) >= 0 }          range of control in mode i
% S_ij = { x | each sX{i}(x) >= 0 } <= X_i   guard in mode i
% XT_i = { x | each hXT{i}(x)>= 0 } <= X_i   target set in mode i
% 
% We solve the following *weak formulation* via relaxation of degree 'd':
% 
% sup  v_i(0,x_0)
% s.t. LFi(v_i) + h_i >= 0
%      v_i(T,x) <= H_i
%      v_i(t,x) <= v_j( t, R_ij (x) )
% ------------------------------------------------------------------------
% 'options' is a struct that contains:
%     .freeFinalTime:   1 = free final time, 0 = fixed final time (default: 0)
%     .withInputs:      1 = perform control synthesis, 0 = o.w. (default: 0)
%     .solver_options:  options that will be passed to SDP solver (default: [])
%     .svd_eps:         svd threshold (default: 1e3)
% ------------------------------------------------------------------------
% Attention: T = 1 is fixed number. To solve OCP for different time
% horizons, try scaling the dynamics and cost functions.
% 

%% Sanity check
if mod(d,2) ~= 0
    warning('d is not even. Using d+1 instead.');
    d = d+1;
end
nmodes = length(x);

max_m = 0;
for i = 1 : nmodes
    m = length(u{i});
    if m > max_m
        max_m = m;
    end
    if (length(f{i}) ~= length(x{i})) || (size(g{i},1) ~= length(x{i}))
        error('Inconsistent matrix size.');
    end
    if size(g{i},2) ~= m
        error('Inconsistent matrix size.');
    end
end

svd_eps = 1e3;
if nargin < 15, options = struct(); end
if isfield(options, 'svd_eps'), svd_eps = options.svd_eps; end
if ~isfield(options, 'freeFinalTime'), options.freeFinalTime = 0; end
if ~isfield(options, 'withInputs'), options.withInputs = 0; end

T = 1;
hT = t*(T-t);

if isempty(R)
    R = cell(nmodes,nmodes);
    for i = 1 : nmodes
    for j = 1 : nmodes
        if ~isempty(sX{i,j}), R{i,j} = x{i}; end
    end
    end
end

%% Iterate through sequence of transitions
out = struct;
out( length(seq) ).u = [];
out( length(seq) ).sol = [];
out( length(seq) ).pval = [];

for i = 1 : length( seq )
    if (i == 1)
        Mu0_basis = monomials( [t; x{ seq(i) }], 0:d );
        Mu0_moments = double( subs( Mu0_basis, [ t; x{ seq(i) } ], [ 0; x0{ seq(i) } ] ) );
        Rst_func = x{ seq(i) };
    else
        Rst_func = R{ seq(i-1), seq( i ) };
    end
    
    if (i == length( seq ))
        [ out2 ] = OCP_singlemode( seq(i), Mu0_basis, Mu0_moments, Rst_func, hXT{ seq(i) }, H{ seq(i) } );
    else
        [ out2 ] = OCP_singlemode( seq(i), Mu0_basis, Mu0_moments, Rst_func, sX{ seq(i), seq(i+1) }, msspoly( 0 ) );
    end
    
    Mu0_basis = out2.muT_basis;
    Mu0_moments = double( out2.y_muT );
    out(i).u = out2.u;
    out(i).sol = out2.sol;
    out(i).pval = out2.pval;
end


%% ================== Solver for each non-hybrid trajectory ==============
    function [ out2 ] = OCP_singlemode( current_mode, mu0_basis, mu0_moments, rst_func, hX_terminal, H_terminal )
        xvar = x{ current_mode };
        uvar = u{ current_mode };
        
        % setup spotless program
        prog = spotsosprog;
        prog = prog.withIndeterminate( t );
        prog = prog.withIndeterminate( xvar );
        prog = prog.withIndeterminate( uvar );
        
        % create decision variable: v
        vmonom = monomials( [t; xvar], 0:d );
        [ prog, v ] = prog.newFreePoly( vmonom );
        
        % create auxiliary variables
        dvdt = diff( v, t );
        dvdx = diff( v, xvar );
        Lfv = dvdt + dvdx * f{ current_mode };
        Lgv = dvdx * g{ current_mode };
        Lv = Lfv + Lgv * uvar;
        
        % constraints
        % 1. Lv + h >= 0                    Dual: mu
        prog = sosOnK( prog, Lv + h{current_mode}, ...
              	[ t; xvar; uvar ], [ hT; hX{current_mode}; hU{current_mode} ], d );
        mu_idx = size( prog.sosExpr, 1 );
        % 2. v(t,x) <= H(x) on X_terminal   Dual: muT
        prog = sosOnK( prog, H_terminal - v, ...
                [ t; xvar ], [ hT; hX_terminal ], d );
        muT_idx = size( prog.sosExpr, 1 );
        
        % objective function
        obj = Ly( subs( v, xvar, rst_func ), mu0_basis, mu0_moments );
        
        
        % set options
        spot_options = spot_sdp_default_options();
        spot_options.verbose = 1;

        if isfield(options, 'solver_options')
            spot_options.solver_options = options.solver_options;
        end
        
        %% solve!
        [sol, y, dual_basis] = prog.minimize( -obj, @spot_mosek, spot_options );
        
        %% generate output
        out2.sol = sol;
        out2.pval = double(sol.eval(obj));
        
        %% control synthesis
        if ~options.withInputs
            out2.u = [];
            return;
        end
        
        urb = monomials( [ t; xvar ], 0 : d/2 );
        % moments of mu
        mu_basis = dual_basis{ mu_idx };
        y_mu = sol.dualEval( y{ mu_idx } );

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

        fprintf('norm of moment matrix %0.2f\n', norm(S));
        fprintf('norm of inverse moment matrix %0.2f\n', norm(pinv(iS1)));

        iMyt = V*pinv(iS1)*U';

        % yp and yn
        for jr = 1 : length( uvar )
            fprintf('Processing mode %1.0f, input #%1.0f ...\n', current_mode, jr );
            mypoly = uvar( jr ) * urb;
            coeff = DecompMatch( mypoly, mu_basis );
            y_u = sol.dualEval( coeff * y_mu );

            u_coeff = iMyt * y_u;
            uout{ jr } = u_coeff' * urb;
        end
        
        out2.u = uout;
        
        %% terminal measure
        muT_basis = dual_basis{ muT_idx };
        y_muT = sol.dualEval( y{ muT_idx } );
        
        out2.muT_basis = muT_basis;
        out2.y_muT = y_muT;
    end

    function [ result ] = Ly( p, basis, moments )
        if numel( p ) > 1
            error( 'Please pass in one polynomial at a time.' );
        end
        [ var, pow, M ] = decomp( p );
        [ basis_var, basis_pow, basis_M ] = decomp( basis );
        basis_pow = basis_M * basis_pow;
        idx1 = match( var, basis_var );     % position of basis_var in var
        
%         p_list = recomp( var( idx1 ), pow( :, idx1 ), speye( size( pow, 1 ) ) );
%         idx2 = match( basis, p_list );
        
        [ ~, idx2 ] = ismember( pow( :, idx1 ), basis_pow, 'rows' );   % position of pow in basis_pow
        moments = [ 0; moments ];
        M = M .* moments( idx2+1 ).';
        
        idx_complement = setdiff( 1:length(var), idx1 );
        pow = pow( :, idx_complement );
        var = var( idx_complement );
        result = recomp( var, pow, M );
    end
end