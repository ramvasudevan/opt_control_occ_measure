% Shortcut problem: 2 modes, 1 input
% Mode 1:
% xdot = [ 0.5; V ]
% Mode 2:
% xdot = 10
% 

clear;
scaling = 4;
d = 6;
nmodes = 2;

% Define variables
t = msspoly( 't', 1 );
ua = msspoly( 'u', 1 );
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

x0{1} = [ -0.8; 0.8 ];

% Dynamics
% Mode 1
x{1} = msspoly( 'xa', 2 );
u{1} = ua;
y = x{1};
f{1} = scaling * [ 0.5;
                   0 ];
g{1} = scaling * [ 0;
                   1 ];

% Mode 2
x{2} = msspoly( 'xb', 1 );
u{2} = ua;
f{2} = scaling * [ 5 ];
g{2} = scaling * [ 0 ];

% Domains
% Mode 1
y = x{1};
hX{1} = 1 - y.^2;
hXT{1} = [ - (y(1) - 0.8)^2;
           - (y(2) + 0.8)^2 ];
sX{1,2} = [ 1 - y(1)^2;
            -(y(2) - 1)^2 ];
R{1,2} = y(1);
h{1} = 1;
H{1} = 0;

% Mode 2
y = x{2};
hX{2} = 1 - y.^2;
sX{2,1} = - (y-1)^2;
R{2,1} = [ 0.6; -0.8 ];
h{2} = 1;
H{2} = 0;

% options
options.MinimumTime = 1;
options.withInputs = 1;

% Solve
% [out] = HybridOptimalControlDualSolver1(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);
[out] = HybridOptimalControlDualSolver(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);

pval = scaling * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
if ~options.withInputs
    return;
end

% Part I
% trajectory from simulation
figure(1);
hold on;
controller = @(tt,xx) [ 1; double(subs(out.u{1,1},[t;x{1}],[tt;xx])) ];
ode_options = odeset('Events',@EventFcn);
[ tval, xval ] = ode45( @(tt,xx) scaling*LinEq( tt, xx, controller ), [0:0.001:1], x0{1}, ode_options );
plot(xval(:,1), xval(:,2),'LineWidth',4);

% control action from simulation
figure(2);
hold on;
uval = zeros( length(tval), 1 );
for i = 1 : length(tval)
    tt = tval(i);
    xx = xval(i,1:2)';
    uval(i) = double( subs(out.u{1,1}, [t;x{1}], [tt;xx]) );
end
plot(tval, uval);


% Part II
figure(1);
controller = @(tt,xx) [ 1; double(subs(out.u{1,1},[t;x{1}],[tt;xx])) ];
ode_options = odeset('Events',@EventFcn);
[ tval, xval ] = ode45( @(tt,xx) scaling*LinEq( tt, xx, controller ), [0.1350:0.001:out.pval], [0.6;-0.8], ode_options );
plot(xval(:,1), xval(:,2),'LineWidth',4);

% control action from simulation
figure(2);
uval = zeros( length(tval), 1 );
for i = 1 : length(tval)
    tt = tval(i);
    xx = xval(i,1:2)';
    uval(i) = double( subs(out.u{1,1}, [t;x{1}], [tt;xx]) );
end
plot(tval, uval);