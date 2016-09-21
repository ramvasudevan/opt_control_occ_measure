function [out] = HybridOptimalControl_Lasserre(t,x,u,f,g,hX,guard,x0,target,d,options)

mset('yalmip',true); % if true Gloptipoly calls the SDP solver through Yalmip
warning('off','YALMIP:strict')

T = 1;
n = length(x);
m = length(u);
nmodes = length(f);
F = cell(nmodes,1);
sym_cnt = 0;

for i = 1 : nmodes
    F{i} = f{i} + g{i} * u;
end

%% Define measures
% mu
mu = cell(nmodes,1);
mu_idx = zeros(nmodes,1);
% mu_idx = sym_cnt + 1;
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
% muT_idx = sym_cnt + 1;
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

% % mu0
% mu0 = cell(nmodes,1);
% mu0_idx = cell(nmodes,1);
% % mu0_idx = sym_cnt + 1;
% for i = 1 : nmodes
%     if ~isempty(x0{i})
%         sym_cnt = sym_cnt + 1;
%         mu0_idx{i} = sym_cnt;
%         mpol( ['symt',num2str(sym_cnt)], 1 );
%         mpol( ['symx',num2str(sym_cnt)], n );
%         mu0{i} = meas(eval(['symt',num2str(sym_cnt)]),...
%                       eval(['symx',num2str(sym_cnt)]));
%     end
% end

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
        if options.MinimumTime
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

% mu0
% for i = 1 : nmodes
%     if ~isempty(x0{i})
%         idx = mu0_idx{i};
%         sym_t = eval(['symt',num2str(idx)]);
%         sym_x = eval(['symx',num2str(idx)]);
%         SptCons = [ SptCons;
%                     sym_t == 0;
%                     sym_x == x0{i} ];
% %         SptCons = [ SptCons;
% %                     sym_t >= 0;
% %                     -sym_t >= 0;
% %                     sym_x - x0{i} >= 0;
% %                     x0{i} - sym_x >= 0 ];
%     end
% end

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

%% Moment constraints
MomCons = [];
for i = 1 : nmodes
    Exp = 0;
    
    % mu
    sym_t = eval(['symt',num2str(mu_idx(i))]);
    sym_x = eval(['symx',num2str(mu_idx(i))]);
    sym_u = eval(['symu',num2str(mu_idx(i))]);
    v = mmon([t;x],d);
    LFv = diff( v, t ) + diff( v, x ) * F{i};
    Exp = Exp + mom(subs(LFv, [t;x;u], [sym_t;sym_x;sym_u]));
    
    % mu0
    if ~isempty(x0{i})
%         idx = mu0_idx{i};
%         sym_t = eval(['symt',num2str(idx)]);
%         sym_x = eval(['symx',num2str(idx)]);
%         Exp = Exp + mom(subs(v, [t;x], [sym_t; sym_x]));
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
            Exp = Exp + mom(subs(v, [t;x], [sym_t; sym_x]));
        end
    end
    
    MomCons = [ MomCons; Exp==0 ];
end

%% Solve
if options.MinimumTime
    obj = 0;
    for i = 1 : nmodes
        obj = obj + mass(mu{i});
    end
end

P = msdp(min(obj), MomCons, SptCons);

SDPsolver = 'mosek';
options = getSolverParams(SDPsolver);
if (~isempty(options))
mset(options);
end

[status,pval] = msol(P);

out.status = status;
out.pval = pval;

