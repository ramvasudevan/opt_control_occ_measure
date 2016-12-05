% SLIP model, 3 modes, vertical partation of flight phase
% Goal: reach a certain distance within minimum time
% h = 0, H = -y, XT = {x >= 4/6/8}
% Free terminal time

clear;

addpath('Utils');
addpath('PolyDynamics');
addpath('TrueDynamics');
addpath('Plot');

d = 6;
scaling = 6;
nmodes = 3;

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
alpha = params.alpha;
yR = params.l0 * cos(alpha);

% Dynamics
domain_size = params.domain_size;
scale_x = cell( nmodes, 1 );
trans_x = cell( nmodes, 1 );
for i = 1 : nmodes
    scale_x{i} = (domain_size{i}(:,2) - domain_size{i}(:,1)) / 2;
    trans_x{i} = mean(domain_size{i},2);
end

x{1} = msspoly( 'xa', 5 );
u{1} = msspoly( 'ua', 1 );
x{2} = msspoly( 'xb', 4 );
u{2} = u{1};
x{3} = x{2};
u{3} = u{2};

f{1} = scaling * rescale_dynamics(@(xx) Stance_f_Approx(xx), x{1}, scale_x{1}, trans_x{1}, scale_x{1} );
g{1} = scaling * rescale_dynamics(@(xx) Stance_g_Approx(xx), x{1}, scale_x{1}, trans_x{1}, scale_x{1} );
for i = 2 : 3
    f{i} = scaling * rescale_dynamics(@(xx) Flight_f(xx), x{i}, scale_x{i}, trans_x{i}, scale_x{i} );
    g{i} = scaling * rescale_dynamics(@(xx) Flight_g(xx), x{i}, scale_x{i}, trans_x{i}, scale_x{i} );
end

% Supports & Guards
% Mode 1 : Stance
hX1 = @(xx) [ domain_size{1}(:,2) - xx;
             xx - domain_size{1}(:,1) ];
G12 = @(xx) [ -(1 - xx(1))^2;         % l = l0
             xx(2);                  % l_dot > 0
             domain_size{1}(3:5,2) - xx(3:5);
             xx(3:5) - domain_size{1}(3:5,1) ];
% Only R12 is needed, because R13 is impossible
hX{1} = rescale_dynamics(hX1, x{1}, scale_x{1}, trans_x{1});
sX{1,2} = rescale_guard(G12, x{1}, scale_x{1}, trans_x{1});
R{1,2} = rescale_reset(@Reset_S2F_Approx, x{1}, scale_x{1}, trans_x{1}, scale_x{2}, trans_x{2});
% Make sure the reset maps to a point in the domain
sX{1,2} = [ sX{1,2};
            1 - R{1,2};
            1 + R{1,2} ];
h{1} = 0;
TarPt1 = @(xx) -xx(1) * ( 1 - xx(3)^2 );	% -l*cos(theta)
H{1} = rescale_guard(TarPt1, x{1}, scale_x{1}, trans_x{1});

% Mode 2 : Flight 1
hX2 = @(xx) [ domain_size{2}(:,2) - xx;
              xx - domain_size{2}(:,1) ];
G23 = @(xx) [ domain_size{2}(1:3,2) - xx(1:3);      % Don't be naive! R23 isn't identity!!
              xx(1:3) - domain_size{2}(1:3,1);
              - xx(4)^2 ];        % y_dot = 0
R23 = @(x) x;
hX{2} = rescale_dynamics(hX2, x{2}, scale_x{2}, trans_x{2});
sX{2,3} = rescale_guard(G23, x{2}, scale_x{2}, trans_x{2});
R{2,3} = rescale_reset(R23, x{2}, scale_x{2}, trans_x{2}, scale_x{3}, trans_x{3});
% Make sure the reset maps to a point in the domain
sX{2,3} = [ sX{2,3};
            1 - R{2,3};
            1 + R{2,3} ];
h{2} = 0;
TarPt2 = @(xx) -xx(3);
H{2} = rescale_guard(TarPt2, x{2}, scale_x{2}, trans_x{2});

% Mode 3 : Flight 2
hX3 = @(xx) [ domain_size{3}(:,2) - xx;
             xx - domain_size{3}(:,1) ];
G31 = @(xx) [ domain_size{3}(1:2,2) - xx(1:2);
              xx(1:2) - domain_size{3}(1:2,1);
              - (xx(3) - yR)^2;
              domain_size{3}(4,2) - xx(4);
              xx(4) - domain_size{3}(4,1) ];
% R31 is Reset_F2S_Approx
hX{3} = rescale_dynamics(hX3, x{3}, scale_x{3}, trans_x{3});
sX{3,1} = rescale_guard(G31, x{3}, scale_x{3}, trans_x{3});
R{3,1} = rescale_reset(@Reset_F2S_Approx, x{3}, scale_x{3}, trans_x{3}, scale_x{1}, trans_x{1});
sX{3,1} = [ sX{3,1};
            1 - R{3,1};
            1 + R{3,1} ];
h{3} = 0;
TarPt3 = @(xx) -xx(3);
H{3} = rescale_guard(TarPt3, x{3}, scale_x{3}, trans_x{3});

% Initial condition and target point
x0{3} = [ 0; 1.7; 1; 0 ];
x0{3} = rescale_state( x0{3}, domain_size{3} );
TarPt1 = @(xx) xx(5) - 9;
TarPt2 = @(xx) xx(1) - 9;
TarPt3 = TarPt2;
hXT{1} = [ hX{1}; rescale_guard(TarPt1, x{1}, scale_x{1}, trans_x{1}) ];
hXT{2} = [ hX{2}; rescale_guard(TarPt2, x{2}, scale_x{2}, trans_x{2}) ];
hXT{3} = [ hX{3}; rescale_guard(TarPt3, x{3}, scale_x{3}, trans_x{3}) ];

% Options
options.MinimumTime = 0;
options.freeFinalTime = 1;
options.withInputs = 1;
options.svd_eps = 1e4;

%% Solve
[out] = HybridOptimalControlDualSolver1(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);

out.pval = out.pval * scaling;

disp(['LMI ' int2str(d) ' lower bound = ' num2str(out.pval)]);
