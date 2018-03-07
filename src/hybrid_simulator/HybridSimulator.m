function [out] = HybridSimulator(t,x,u,f,g,hX,sX,R,x0,d,dr,options)
% u is a msspoly of t and x

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

%% Setup spotless program
% define transition state variable
r = msspoly( 'r', 1 );
rmonom = monomials( r, 0:dr );

% define the program
prog = spotsosprog;
prog = prog.withIndeterminate( t );
prog = prog.withIndeterminate( r );

% constraints to keep track of
mu_idx = zeros(nmodes, 1);

% Create program variables in each mode
for i = 1 : nmodes
    
    prog = prog.withIndeterminate( x{i} );
    
    % create v(i)
    txmonom = monomials( [t; x{i}], 0:d );
    vmonom{ i } = PolyCombine( txmonom, rmonom );
    [ prog, v{ i }, ~ ] = prog.newFreePoly( vmonom{ i } );
    
    % create the variables that will be used later
    vT{ i } = subs( v{ i }, t, T );
    dvdt{ i } = diff( v{ i }, t );
    dvdx{ i } = diff( v{ i }, [ x{ i }; r ] );
    Lfv{ i } = dvdt{ i } + dvdx{ i } * [ f{ i }; 0 ];
    Lgv{ i } = dvdx{ i } * [ g{ i }; zeros(1, size( g{i}, 2 )) ];
    Lv{ i } = Lfv{ i } + Lgv{ i } * u{ i };
end

% creating the constraints and cost function
obj = 0;
for i = 1 : nmodes
    % Lv_i >= 0                   Dual: mu
    prog = sosOnK( prog, Lv{ i }, ...
                   [ t; x{ i } ], [ hT; hX{ i } ], d);
    mu_idx(i) = size( prog.sosExpr, 1 );
    
    % v(T,x) <= 0                  Dual: muT
%     if ~isempty( hXT{ i } )
%         if options.freeFinalTime
%             prog = sosOnK( prog, - v{ i }, [ t; x{ i } ], [ hT; hX{ i } ], d );
%         else
            prog = sosOnK( prog, - vT{ i }, x{ i }, hX{ i }, d );
%         end
%     end
    
    % v_i(t,x) <= v_j (t,R(x))          Dual: muS
    for j = 1 : nmodes
        if ( ~isempty( sX{ i, j } ) ) % if its empty there isn't a guard between these
            vj_helper = subs( v{ j }, [x{ j }; r], [ R{ i, j }; r+1 ] ); 
            prog = sosOnK_M( prog, vj_helper - v{ i }, [ t; x{ i } ] , [ hT; sX{ i, j } ], d, r, dr );
        end
    end
    
    % Objective function
    if ~isempty( x0{ i } )
        obj = obj + subs( v{ i }, [ t; x{i}; r ], [ 0; x0{i}; 0 ] );
    end
end

% set options
spot_options = spot_sdp_default_options();
spot_options.verbose = 1;

if isfield(options, 'solver_options')
    spot_options.solver_options = options.solver_options;
end

%% Solve
tic;
[sol, y, dual_basis, ~] = prog.minimize( -obj, @spot_mosek, spot_options );
out.time = toc;

out.pval = double(sol.eval(obj));
out.sol = sol;

%% Transition sequence
if ~options.withTransition
    out.trans = [];
    return;
end

trans = cell( nmodes, 1 );

for i = 1 : nmodes
    basis = monomials( r, 0:dr );
    mu_basis = dual_basis{ mu_idx(i) };
    y_mu = sol.dualEval( y{ mu_idx(i) } );
    
    coeff = DecompMatch( basis, mu_basis );
    vec = double( coeff * y_mu );
    
    M = zeros( dr+1, 4 );
    for j = 0 : dr
        M(j+1,:) = (0:3) .^ j;
    end
    trans{i} = M\vec;
end

out.trans = trans;





