% SLIP model, only stance mode, no reset, just to see what the problem is 
% It's different from SimpleModel.m in two ways:
% 1. The target set is just the entire space, and the cost function looks
% something like "h = 0, H = (x - x_target)^2"
% 2. The initial condition is theta = theta_dot = 0, so we don't lean
% forwards or backwards.
% 

clear;

addpath('Utils');
addpath('Dynamics');

d = 6;
scaling = 2;
nmodes = 1;

% polysin = @(x) x - x^3/6;
% polycos = @(x) 1 - x^2 + x^4/24;
% polysin = @(x) x;
% polycos = @(x) 1 - x^2;

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

params = SLIPParams;
M = params.M;
k = params.k;
l0 = params.l0;
gg = params.gg;

% Dynamics
domain_size{1} = [ 0.5, 1;
                   -5, 5;
                   -pi/3, pi/3;
                   -2*pi/3, 2*pi/3;
                   0, 5 ];
for i = 1 : nmodes
    scale_x{i} = (domain_size{i}(:,2) - domain_size{i}(:,1)) / 2;
    trans_x{i} = mean(domain_size{i},2);
end


x{1} = msspoly( 'xa', 5 );
u{1} = msspoly( 'ua', 1 );

f{1} = scaling * rescale_dynamics(@(x) Stance_simp_f(x,params), x{1}, scale_x{1}, trans_x{1}, scale_x{1} );
g{1} = scaling * rescale_dynamics(@(x) Stance_simp_g(x,params), x{1}, scale_x{1}, trans_x{1}, scale_x{1} );

% Supports & Guards
% Mode 1 : Stance
hX1 = @(x) [ domain_size{1}(:,2) - x;
             x - domain_size{1}(:,1) ];

hX{1} = rescale_dynamics(hX1, x{1}, scale_x{1}, trans_x{1});
h{1} = 0;
H{1} = (x{1}(1) - 0.5)^2;
% H{1} = 0;

% Initial condition and target point
x0{1} = [ 0.5; -0.05; 0; 0; 0 ];
% TarPt1 = @(x) [ domain_size{1}(2:5,2) - x(2:5);
%                 x(2:5) - domain_size{1}(2:5,1);
%                 x(1) - 0.9 ];
% hXT{1} = rescale_guard(TarPt1, x{1}, scale_x{1}, trans_x{1});
hXT{1} = hX{1};

% Options
options.MinimumTime = 0;
options.withInputs = 0;
options.svd_eps = 1e4;

%% Solve
[out] = HybridOptimalControlDualSolver1(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);

out.pval = out.pval * scaling;

disp(['LMI ' int2str(d) ' lower bound = ' num2str(out.pval)]);
