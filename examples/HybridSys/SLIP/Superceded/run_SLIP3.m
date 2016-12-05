% SLIP model, 1 mode, 
% It's different from run_SLIP2 in the way that our goal point isn't too far in thic case.
% 

clear;

addpath('Utils');
addpath('Dynamics');

d = 6;
scaling = 10;
nmodes = 1;
params = SLIPParams;

% Define variables
t = msspoly( 't', 1 );
x = cell( nmodes, 1 );
u = cell( nmodes, 1 );
f = cell( nmodes, 1 );
g = cell( nmodes, 1 );
x0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );

domain_size = cell( nmodes, 1 );
scale_x = cell( nmodes, 1 );
trans_x = cell( nmodes, 1 );

% Dynamics
domain_size{1} = [ 0.5, 1;
                   -5, 5;
                   -pi/3, pi/3;
                   -2*pi/3, 0;
                   0, 5 ];

for i = 1 : nmodes
    scale_x{i} =  (domain_size{i}(:,2) - domain_size{i}(:,1)) / 2;
    trans_x{i} = mean(domain_size{i},2);
end

x{1} = msspoly( 'xa', 5 );
u{1} = msspoly( 'ua', 1 );

f{1} = scaling * rescale_dynamics(@(x) Stance_simp_f(x,params), x{1}, scale_x{1}, trans_x{1}, scale_x{1} );
g{1} = scaling * rescale_dynamics(@(x) Stance_simp_g(x,params), x{1}, scale_x{1}, trans_x{1}, scale_x{1} );

% Supports & Guards
hX1 = @(x) [ domain_size{1}(:,2) - x;
             x - domain_size{1}(:,1) ];
G11 = @(x) [ -(1 - x(1))^2;
             x(2);
             domain_size{1}(3:5,2) - x(3:5);
             x(3:5) - domain_size{1}(3:5,1);
             (-1/2).*3.^(1/2)+x(2).^2+(-1).*x(1).*((-2)+x(3).^2)+(-2).*x(2).*x(3).*x(4) ];
R11 = @(x) Reset_simp(x);

hX{1} = rescale_dynamics(hX1, x{1}, scale_x{1}, trans_x{1});
sX{1,1} = rescale_guard(G11, x{1}, scale_x{1}, trans_x{1});
R{1,1} = rescale_reset(R11, x{1}, scale_x{1}, trans_x{1}, scale_x{1}, trans_x{1});
h{1} = 1;
H{1} = 0;

% Initial condition and target point
x0{1} = [ params.l0-0.1;
          -1;
          params.alpha;
          1.5;
          0 ];
TarPt1 = @(x) [ domain_size{1}(1:4,2) - x(1:4);
                x(1:4) - domain_size{1}(1:4,1);
                x(5) - 1.5;
                1.5 - x(5) ];
hXT{1} = rescale_guard(TarPt1, x{1}, scale_x{1}, trans_x{1});

% Options
options.MinimumTime = 1;
options.withInputs = 1;
options.svd_eps = 1e4;

%% Solve
[out] = HybridOptimalControlDualSolver1(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);

out.pval = out.pval * scaling;

disp(['LMI ' int2str(d) ' lower bound = ' num2str(out.pval)]);
