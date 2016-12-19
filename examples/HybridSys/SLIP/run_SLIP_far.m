% SLIP model, 3 modes, vertical partation of flight phase
% Goal: reach a certain distance within minimum time
% h = 1, H = 0, XT = {x >= 4/6/8}
% Free terminal time

clear;

addpath('Utils');
addpath('PolyDynamics');
addpath('TrueDynamics');
addpath('Plot');

d = 8;
scaling = 6;
nmodes = 3;
% Target = -0.4;
Target = 0.5;

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

params = SLIPParams;
l0 = params.l0;
alpha = params.alpha;
yR = params.l0 * cos(alpha);

% Dynamics
domain_size = params.domain_size;
% scale_x = cell( nmodes, 1 );
% trans_x = cell( nmodes, 1 );
% for i = 1 : nmodes
%     scale_x{i} = (domain_size{i}(:,2) - domain_size{i}(:,1)) / 2;
%     trans_x{i} = mean(domain_size{i},2);
% end

x{1} = msspoly( 'xa', 5 );
u{1} = msspoly( 'ua', 1 );
x{2} = msspoly( 'xb', 4 );
u{2} = u{1};
x{3} = x{2};
u{3} = u{2};

f{1} = scaling * Stance_f_Approx(x{1});
g{1} = scaling * Stance_g_Approx(x{1});
for i = 2 : 3
    f{i} = scaling * Flight_f(x{i});
    g{i} = scaling * Flight_g(x{i});
end

% Supports & Guards
% Mode 1 : Stance
y = x{1};
hX{1} = [ domain_size{1}(:,2) - y;
          y - domain_size{1}(:,1) ];
R{1,2} = Reset_S2F_Approx(y);
sX{1,2} = [ -(l0 - y(1))^2;                     % l = l0
             y(2);                              % l_dot > 0
             hX{1};                             % G \subset X
             domain_size{2}(:,2) - R{1,2};      % R(i,j) \subset X_j
             R{1,2} - domain_size{2}(:,1) ];
h{1} = 1;
H{1} = 0;

% Mode 2 : Flight 1
y = x{2};
hX{2} = [ domain_size{2}(:,2) - y;
          y - domain_size{2}(:,1) ];
R{2,3} = y;
sX{2,3} = [ -y(4)^2;                             % y_dot = 0
            hX{2};                               % G \subset X
            domain_size{3}(:,2) - R{2,3};        % R(i,j) \subset X_j
            R{2,3} - domain_size{3}(:,1) ];
h{2} = 1;
H{2} = 0;

% Mode 3 : Flight 2
y = x{3};
hX{3} = [ domain_size{3}(:,2) - y;
          y - domain_size{3}(:,1) ];
R{3,1} = Reset_F2S_Approx(y);
sX{3,1} = [ -(y(3) - yR)^2;                       % y = yR
            hX{3};
            domain_size{1}(:,2) - R{3,1};
            R{3,1} - domain_size{1}(:,1) ];
h{3} = 1;
H{3} = 0;

% Initial condition and target point
x0{3} = [ -1; 0.3; 0.21; 0 ];

hXT{1} = [ x{1}(5) - Target; hX{1} ];
hXT{2} = [ x{2}(1) - Target; hX{2} ];
hXT{3} = [ x{3}(1) - Target; hX{3} ];

% Options
options.MinimumTime = 1;
options.withInputs = 1;
options.svd_eps = 1e4;

%% Solve
[out] = HybridOCPDualSolver(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);

out.pval = out.pval * scaling;

disp(['LMI ' int2str(d) ' lower bound = ' num2str(out.pval)]);
