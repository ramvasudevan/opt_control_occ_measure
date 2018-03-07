% Dubins car: 2 modes
% State variables: [x, y, theta]'
% Input: [ V, u ]'
% System dynamics:
% xdot = [ V*cos(1.5*theta)
%          V*sin(1.5*theta)
%          10 * u ]

clear;
scaling = 3;
d = 6;
nmodes = 1;

% polysin = @(x) x - x^3/6;
% polycos = @(x) 1 - x^2/2 + x^4/24;
polysin = @(x) x;
polycos = @(x) 1 - x^2/2;

% Define variables
t = msspoly( 't', 1 );
xa = msspoly( 'x', 3 );
ua = msspoly( 'u', 2 );
x = cell( nmodes, 1 );
u = cell( nmodes, 1 );
f = cell( nmodes, 1 );
g = cell( nmodes, 1 );
x0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
hU = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );

x0{1} = [ -0.8; 0.8; 0 ];
xT{1} = [ 0.5; -0.4; 0 ];

% Dynamics
% Mode 1
x{1} = xa;
u{1} = ua;
f{1} = scaling * [ 0;
                   0;
                   0 ];
g{1} = scaling * [ polycos(xa(3)), 0;
                   polysin(xa(3)), 0;
                   0,                  3 ];
% Domains
% Mode 1
y = x{1};
hX{1} = [ 1 - y(1)^2;           % X_1 = [-1,1] x [-1,1] x [-pi/2,pi/2]
          1 - y(2)^2;
          (pi/2)^2 - y(3).^2 ];
hU{1} = [ ua(1) * (1 - ua(1));      % U_1 = [0,1] x [-1,1]
          1 - ua(2)^2 ];
hXT{1} = 1 - y.^2;
h{1} = (xa(1)-xT{1}(1))^2 + (xa(2)-xT{1}(2))^2 + ua(1)^2 + ua(2)^2;
H{1} = 0;

% Options
options.freeFinalTime = 0;
options.withInputs = 1;
options.svd_eps = 1e5;

% Solve
[out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);

pval = scaling * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
if ~options.withInputs
    return;
end

figure;
hold on;
% trajectory from simulation
controller = @(tt,xx) [ double(subs(out.u{1,1},[t;xa],[tt;xx])); double(subs(out.u{1,2},[t;xa],[tt;xx])) ];
[ tval, xval ] = ode45( @(tt,xx) scaling*DubinsEq( tt, xx, controller ), [0,out.pval], x0{1} );
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',4);

% control action
figure;
uval = zeros( length(tval), 2 );
for i = 1 : length(tval)
    tt = tval(i);
    xx = xval(i,1:3)';
    uval(i,1) = double( subs(out.u{1,1}, [t;xa], [tt;xx]) );
    uval(i,2) = double( subs(out.u{1,2}, [t;xa], [tt;xx]) );
end
tval = scaling * tval;
subplot(1,2,1);
hold on;
plot(tval,uval(:,1),'LineWidth',4);
plot([0,tval(end)],[1,1],'k');
ylim([0,1.1]);
xlabel('$t$','Interpreter','LaTex','FontSize',20);
ylabel('$V(t)$','Interpreter','LaTex','FontSize',20);
subplot(1,2,2);
hold on;
plot(tval,uval(:,2),'LineWidth',4);
xlabel('$t$','Interpreter','LaTex','FontSize',20);
ylabel('$\omega(t)$','Interpreter','LaTex','FontSize',20);

