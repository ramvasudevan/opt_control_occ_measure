% Shortcut problem: 2 modes
% Mode 1: Dubins car
% System dynamics:
% xdot = [ V*cos(1.5*theta)
%          V*sin(1.5*theta)
%          2 * u ]
% 
% Mode 2: Shortcut
% System dynamics:
% xdot = V
% 

clear;
scaling = 3;
d = 6;
nmodes = 2;

polysin = @(x) x;
polycos = @(x) 1 - x^2/2;

% Define variables
t = msspoly( 't', 1 );
ua = msspoly( 'u', 2 );
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

x0{1} = [ -0.8; 0.8; 0 ];

% Dynamics
% Mode 1
x{1} = msspoly( 'xa', 3 );
u{1} = ua;
y = x{1};
f{1} = scaling * [ 0;
                   0;
                   0 ];
g{1} = scaling * [ polycos(1.5*y(3)), 0;
                   polysin(1.5*y(3)), 0;
                   0,                  2 ];

% Mode 2
x{2} = msspoly( 'xb', 1 );
u{2} = ua;
f{2} = scaling * 0;
g{2} = scaling * [ 10, 0 ];

% Domains
% Mode 1
y = x{1};
hX{1} = 1 - y.^2;
hXT{1} = [ - (y(1) - 0.8)^2;
           - (y(2) + 0.8)^2;
           1 - y(3)^2 ];
sX{1,2} = [ 1 - y(1)^2;
            -(y(2) - 1)^2;
            1 - y(3)^2 ];
R{1,2} = -1;
h{1} = 1;
H{1} = 0;

% Mode 2
y = x{2};
hX{2} = 1 - y.^2;
sX{2,1} = - (y-1)^2;
R{2,1} = [ 0.8; -1; sqrt(2)/1.5 ];
h{2} = 1;
H{2} = 0;

% options
options.MinimumTime = 1;
options.withInputs = 1;

% Solve
[out] = HybridOptimalControlDualSolver1(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);

pval = scaling * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
if ~options.withInputs
    return;
end

figure;
hold on;
% trajectory from simulation
controller = @(tt,xx) [ double(subs(out.u{1,1},[t;x{1}],[tt;xx])); double(subs(out.u{1,2},[t;x{1}],[tt;xx])) ];
ode_options = odeset('Events',@EventFcn);
[ tval, xval ] = ode45( @(tt,xx) scaling*DubinsEq( tt, xx, controller ), [0:0.01:1], x0{1}, ode_options );
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',4);

% control action
figure;
hold on;
uval = zeros( length(tval), 2 );
for i = 1 : length(tval)
    tt = tval(i);
    xx = xval(i,1:3)';
    uval(i,1) = double( subs(out.u{1,1}, [t;x{1}], [tt;xx]) );
    uval(i,2) = double( subs(out.u{1,2}, [t;x{1}], [tt;xx]) );
end
subplot(1,2,1);
plot(tval,uval(:,1));
subplot(1,2,2);
plot(tval,uval(:,2));
% [ tval2, xval2 ] = ode45( @(tt,xx) scaling*DubinsEq( tt, xx, controller ), [tval(end)+0.2/scaling:0.01:pval/scaling], [0.8;-1;sqrt(2)/1.5] );
% plot(xval(:,1),xval(:,2), 'LineWidth',4);