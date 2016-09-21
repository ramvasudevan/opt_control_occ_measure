function [out] = HybridOptimalControl_withInputs(t,x,u,f,g,hX,guard,R,x0,target,objective,d,options)

mset('yalmip',true); % if true Gloptipoly calls the SDP solver through Yalmip
warning('off','YALMIP:strict')

if mod(d,2) ~= 0
    warning('d is not even. Using d+1 instead.');
    d = d+1;
end

if ~isfield(options, 'freeFinalTime'), options.freeFinalTime = 0; end
if ~isfield(options, 'withInputs'), options.withInputs = 0; end
if ~isfield(options, 'MinimumTime'), options.MinimumTime = 0; end
if options.MinimumTime == 1
    options.freeFinalTime = 1;
end

T = 1;
n = length(x);
m = length(u);
nmodes = length(f);
F = cell(nmodes,1);
sym_cnt = 0;

if isempty(R)
    R = cell(nmodes,nmodes);
    for i = 1 : nmodes
    for j = 1 : nmodes
        if ~isempty(guard{i,j}), R{i,j} = x; end
    end
    end
end

% Determinant of R
D = cell(nmodes, nmodes);
for i = 1 : nmodes
    for j = 1 : nmodes
        if ~isempty(guard{i,j})
            if isempty(R{i,j})
                error('Reset map error!!');
            end
            D{i,j} = det(diff(R{i,j},x));
        end
    end
end

for i = 1 : nmodes
    F{i} = f{i} + g{i} * u;
end

%% Define measures
% mu
mu = cell(nmodes,1);
mu_idx = zeros(nmodes,1);
for i = 1 : nmodes
    sym_cnt = sym_cnt + 1;
    mu_idx(i) = sym_cnt;
    mpol( ['symt',num2str(sym_cnt)], 1 );
    mpol( ['symx',num2str(sym_cnt)], n );
    mpol( ['symu',num2str(sym_cnt)], m );
    mu{i} = meas(eval(['symt',num2str(sym_cnt)]),...
                 eval(['symx',num2str(sym_cnt)]),...
                 eval(['symu',num2str(sym_cnt)]));
end

% muT
muT = cell(nmodes,1);
muT_idx = zeros(nmodes,1);
for i = 1 : nmodes
    if ~isempty(target{i})
        sym_cnt = sym_cnt + 1;
        muT_idx(i) = sym_cnt;
        mpol( ['symt',num2str(sym_cnt)], 1 );
        mpol( ['symx',num2str(sym_cnt)], n );
        muT{i} = meas(eval(['symt',num2str(sym_cnt)]),...
                      eval(['symx',num2str(sym_cnt)]));
    end
end

% Guard
muG = cell(nmodes,nmodes);
muG_idx = zeros(nmodes,nmodes);
for i = 1 : nmodes
    for j = 1 : nmodes
        if ~isempty(guard{i,j})
            sym_cnt = sym_cnt + 1;
            muG_idx(i,j) = sym_cnt;
            mpol( ['symt',num2str(sym_cnt)], 1 );
            mpol( ['symx',num2str(sym_cnt)], n );
            muG{i,j} = meas(eval(['symt',num2str(sym_cnt)]),...
                            eval(['symx',num2str(sym_cnt)]));
        end
    end
end

% gamma
gamma = cell(nmodes,1);
gamma_idx = zeros(nmodes,1);
for i = 1 : nmodes
    sym_cnt = sym_cnt + 1;
    gamma_idx(i) = sym_cnt;
    mpol( ['symt',num2str(sym_cnt)], 1 );
    mpol( ['symx',num2str(sym_cnt)], n );
    gamma{i} = meas(eval(['symt',num2str(sym_cnt)]),...
                    eval(['symx',num2str(sym_cnt)]));
end

% sigmaP, sigmaN, sigmaH
sp = cell(nmodes,m);
sn = cell(nmodes,m);
sh = cell(nmodes,m);
sigma_idx = zeros(nmodes,m);
for i = 1 : nmodes
    for j = 1 : m
        sym_cnt = sym_cnt + 1;
        sigma_idx(i,j) = sym_cnt;
        % sp
        mpol( ['symt',num2str(sym_cnt)], 1 );
        mpol( ['symx',num2str(sym_cnt)], n );
        sp{i,j} = meas(eval(['symt',num2str(sym_cnt)]),...
                       eval(['symx',num2str(sym_cnt)]));
        % sn
        sym_cnt = sym_cnt + 1;
        mpol( ['symt',num2str(sym_cnt)], 1 );
        mpol( ['symx',num2str(sym_cnt)], n );
        sn{i,j} = meas(eval(['symt',num2str(sym_cnt)]),...
                       eval(['symx',num2str(sym_cnt)]));
        % sh
        sym_cnt = sym_cnt + 1;
        mpol( ['symt',num2str(sym_cnt)], 1 );
        mpol( ['symx',num2str(sym_cnt)], n );
        sh{i,j} = meas(eval(['symt',num2str(sym_cnt)]),...
                       eval(['symx',num2str(sym_cnt)]));
    end
end



%% Spt constraints
SptCons = [];
% mu
for i = 1 : nmodes
    sym_t = eval(['symt',num2str(mu_idx(i))]);
    sym_x = eval(['symx',num2str(mu_idx(i))]);
    sym_u = eval(['symu',num2str(mu_idx(i))]);
    SptCons = [ SptCons;
                sym_t * (T - sym_t) >= 0;
                subs(hX{i}, x, sym_x) >= 0;
                1 - sym_u .^ 2 >= 0 ];
end

% muT
for i = 1 : nmodes
    if ~isempty(target{i})
        idx = muT_idx(i);
        sym_t = eval(['symt',num2str(idx)]);
        sym_x = eval(['symx',num2str(idx)]);
        if options.freeFinalTime
            SptCons = [ SptCons;
                        sym_t * (T - sym_t) >= 0;
                        subs(target{i}, x, sym_x) >= 0 ];
        else
            SptCons = [ SptCons;
                        sym_t == T;
                        subs(hX{i}, x, sym_x) >= 0 ];
        end
    end
end

% muG
for i = 1 : nmodes
    for j = 1 : nmodes
        if ~isempty(guard{i,j})
            idx = muG_idx(i,j);
            sym_t = eval(['symt',num2str(idx)]);
            sym_x = eval(['symx',num2str(idx)]);
            SptCons = [ SptCons;
                        sym_t * (T - sym_t) >= 0;
                        subs(guard{i,j}, x, sym_x) >= 0 ];
        end
    end
end

% gamma
for i = 1 : nmodes
    sym_t = eval(['symt',num2str(gamma_idx(i))]);
    sym_x = eval(['symx',num2str(gamma_idx(i))]);
    SptCons = [ SptCons;
                sym_t * (T - sym_t) >= 0;
                subs(hX{i}, x, sym_x) >= 0 ];
end

% sigmaP, sigmaN, sigmaH
for i = 1 : nmodes
    for j = 1 : m
        % sp
        sym_t = eval(['symt',num2str(sigma_idx(i,j))]);
        sym_x = eval(['symx',num2str(sigma_idx(i,j))]);
        SptCons = [ SptCons;
                    sym_t * (T - sym_t) >= 0;
                    subs(hX{i}, x, sym_x) >= 0 ];
        % sn
        sym_t = eval(['symt',num2str(sigma_idx(i,j)+1)]);
        sym_x = eval(['symx',num2str(sigma_idx(i,j)+1)]);
        SptCons = [ SptCons;
                    sym_t * (T - sym_t) >= 0;
                    subs(hX{i}, x, sym_x) >= 0 ];
        % sh
        sym_t = eval(['symt',num2str(sigma_idx(i,j)+2)]);
        sym_x = eval(['symx',num2str(sigma_idx(i,j)+2)]);
        SptCons = [ SptCons;
                    sym_t * (T - sym_t) >= 0;
                    subs(hX{i}, x, sym_x) >= 0 ];
    end
end

%% Moment constraints
MomCons = [];

v = mmon([t;x],d);
dvdt = diff( v, t );
dvdx = diff( v, x );
Lfv = cell(nmodes,1);
Lgv = cell(nmodes,m);
LFv = cell(nmodes,1);
for i = 1 : nmodes
    Lfv{i} = dvdt + dvdx * f{i};
    for j = 1 : m
        Lgv{i,j} = dvdx * g{i}(:,j);
    end
    LFv{i} = dvdt + dvdx * F{i};
end


% Constraint 1 : mu0(i) + LF'mu(i) + muG(j->i) - muG(i->j) - muT(i) == 0
for i = 1 : nmodes
    Exp = 0;
    
    % mu
    sym_t = eval(['symt',num2str(mu_idx(i))]);
    sym_x = eval(['symx',num2str(mu_idx(i))]);
    sym_u = eval(['symu',num2str(mu_idx(i))]);
    Exp = Exp + mom(subs(LFv{i}, [t;x;u], [sym_t;sym_x;sym_u]));
    
    % mu0
    if ~isempty(x0{i})
        Exp = Exp + double( subs( v, [t;x], [0;x0{i}] ) );
    end
    
    % muT
    if ~isempty(target{i})
        idx = muT_idx(i);
        sym_t = eval(['symt',num2str(idx)]);
        sym_x = eval(['symx',num2str(idx)]);
        Exp = Exp - mom(subs(v, [t;x], [sym_t; sym_x]));
    end
    
    % muG
    for j = 1 : nmodes
        if ~isempty(guard{i,j})
            idx = muG_idx(i,j);
            sym_t = eval(['symt',num2str(idx)]);
            sym_x = eval(['symx',num2str(idx)]);
            Exp = Exp - mom(subs(v, [t;x], [sym_t; sym_x]));
        end
        if ~isempty(guard{j,i})
            idx = muG_idx(j,i);
            sym_t = eval(['symt',num2str(idx)]);
            sym_x = eval(['symx',num2str(idx)]);
            integrand = subs(v, x, R{j,i}) * D{j,i};
            Exp = Exp + mom(subs(integrand, [t;x], [sym_t; sym_x]));
        end
    end
    
    MomCons = [ MomCons; Exp==0 ];
end

% Constraint 2 : LF'mu(i) = Lf'gamma(i) + sum_{j=1}^m { Lg'(i,j) * [sp(i,j) - sn(i,j)] }
for i = 1 : nmodes
    Exp = 0;
    
    % mu
    sym_t = eval(['symt',num2str(mu_idx(i))]);
    sym_x = eval(['symx',num2str(mu_idx(i))]);
    sym_u = eval(['symu',num2str(mu_idx(i))]);
    Exp = Exp + mom(subs(LFv{i}, [t;x;u], [sym_t;sym_x;sym_u]));
    
    % gamma
    sym_t = eval(['symt',num2str(gamma_idx(i))]);
    sym_x = eval(['symx',num2str(gamma_idx(i))]);
    Exp = Exp - mom(subs(Lfv{i}, [t;x], [sym_t;sym_x]));
    
    % sigma
    for j = 1 : m
        % sp
        sym_t = eval(['symt',num2str(sigma_idx(i,j))]);
        sym_x = eval(['symx',num2str(sigma_idx(i,j))]);
        Exp = Exp - mom(subs(Lgv{i,j}, [t;x], [sym_t;sym_x]));
        % sn
        sym_t = eval(['symt',num2str(sigma_idx(i,j)+1)]);
        sym_x = eval(['symx',num2str(sigma_idx(i,j)+1)]);
        Exp = Exp + mom(subs(Lgv{i,j}, [t;x], [sym_t;sym_x]));
    end
    
    MomCons = [ MomCons; Exp==0 ];
end

% Constraint 3 : sp(i,j) + sn(i,j) + sh(i,j) - gamma(i) == 0
for i = 1 : nmodes
    sym_t = eval(['symt',num2str(gamma_idx(i))]);
    sym_x = eval(['symx',num2str(gamma_idx(i))]);
    Exp_gamma = mom( mmon([sym_t;sym_x], d) );
    for j = 1 : m
        Exp = - Exp_gamma;
        
        % sp
        sym_t = eval(['symt',num2str(sigma_idx(i,j))]);
        sym_x = eval(['symx',num2str(sigma_idx(i,j))]);
        Exp = Exp + mom( mmon([sym_t;sym_x], d) );
        % sn
        sym_t = eval(['symt',num2str(sigma_idx(i,j)+1)]);
        sym_x = eval(['symx',num2str(sigma_idx(i,j)+1)]);
        Exp = Exp + mom( mmon([sym_t;sym_x], d) );
        % sh
        sym_t = eval(['symt',num2str(sigma_idx(i,j)+2)]);
        sym_x = eval(['symx',num2str(sigma_idx(i,j)+2)]);
        Exp = Exp + mom( mmon([sym_t;sym_x], d) );
        
        MomCons = [ MomCons; Exp == 0 ];
    end
end

% Constraint 4 : (t,x)-marginal of mu(i) - gamma(i) == 0
for i = 1 : nmodes
    Exp = 0;
    % mu
    sym_t = eval(['symt',num2str(mu_idx(i))]);
    sym_x = eval(['symx',num2str(mu_idx(i))]);
    Exp = Exp + mom( mmon([sym_t;sym_x], d) );
    % gamma
    sym_t = eval(['symt',num2str(gamma_idx(i))]);
    sym_x = eval(['symx',num2str(gamma_idx(i))]);
    Exp = Exp - mom( mmon([sym_t;sym_x], d) );
    
    MomCons = [ MomCons; Exp == 0 ];
end


%% Solve
if options.MinimumTime
    obj = 0;
    for i = 1 : nmodes
        obj = obj + mass(mu{i});
    end
else
    obj = 0;
    for i = 1 : nmodes
        sym_t = eval(['symt',num2str(mu_idx(i))]);
        sym_x = eval(['symx',num2str(mu_idx(i))]);
        sym_u = eval(['symu',num2str(mu_idx(i))]);
        obj = obj + mom( subs(objective{i},[t;x;u],[sym_t;sym_x;sym_u]) );
    end
end

Prob = msdp(min(obj), MomCons, SptCons);

SDPsolver = 'mosek';
YalmipOptions = getSolverParams(SDPsolver);
if (~isempty(YalmipOptions))
mset(YalmipOptions);
end

[status,pval] = msol(Prob);

out.status = status;
out.pval = pval;

%% Extract control input
if options.withInputs
    Ctrl = cell(nmodes,m);
    for i = 1 : nmodes
        % Moment matrix of gamma
        tgamma = eval(['symt',num2str(gamma_idx(i))]);
        xgamma = eval(['symx',num2str(gamma_idx(i))]);
        phi = mmon([ tgamma; xgamma ], d/2 );
        cnt = 1;
        len = length(phi);
        P = mpol( zeros( len*(len+1)/2, 1 ) );
        for a = 1 : length(phi)
            for b = 1 : a
                P(cnt) = phi(a) * phi(b);
                cnt = cnt + 1;
            end
        end
        M = mss_v2s(double(mom(P)));
        
        % Bad condition of M
        [U,S,V] = svd(M);
        iS1 = S;
        
        startExp = 1e-10;
        while norm(pinv(iS1)) / norm(S) > 1000
            iS1(iS1 < startExp) = 0;
            startExp = startExp * 10;
        end
        
        disp(sprintf('norm of moment matrix %0.2f', norm(S)))
        disp(sprintf('norm of inverse moment matrix %0.2f', norm(pinv(iS1))))
        
        iMyt = V*pinv(iS1)*U';
        
        % For each input
        for j = 1 : m
            tsp = eval(['symt',num2str(sigma_idx(i,j))]);
            xsp = eval(['symx',num2str(sigma_idx(i,j))]);
            tsn = eval(['symt',num2str(sigma_idx(i,j)+1)]);
            xsn = eval(['symx',num2str(sigma_idx(i,j)+1)]);
            
            phi_sp = mmon([ tsp; xsp ], d/2);
            phi_sn = mmon([ tsn; xsn ], d/2);
            y = double( mom(phi_sp) ) - double( mom(phi_sn) );
            
            vec = iMyt * y;
            Ctrl{i,j} = vec' * mmon( [t;x], d/2 );
        end
    end
    
    out.u = Ctrl;
else
    out.u = [];
end
