% Dubins car - Minimum Time, using classical version of OCP code
% State variables: [x, y, theta]'
% Input: [ V, u ]'
% System dynamics:
% xdot = [ V*cos(1.5*theta)
%          V*sin(1.5*theta)
%          2 * u ]

clear;
scaling = 3;
degree = 8;

% polysin = @(x) x - x^3/6;
% polycos = @(x) 1 - x^2/2 + x^4/24;
polysin = @(x) x;
polycos = @(x) 1 - x^2/2;

% dynamics
t = msspoly( 't', 1 );
x = msspoly( 'x', 3 );
u = msspoly( 'u', 2 );
f = scaling * [ 0;
                0;
                0 ];
g = scaling * [ polycos(1.5*x(3)), 0;
                polysin(1.5*x(3)), 0;
                0,                  2 ];

x0 = [ -0.8; 0.8; 0 ];
xT = [ 0.5; -0.2; 0 ];
hX = 1 - x.^2;
hXT = - ( x - xT ).^2;
h = 1;
H = 0;

% options
options.MinimumTime = 1;
options.withInputs = 1;
options.svd_eps = 1e5;

% Solve
[out] = OCPDualSolver( t, x, u, f, g, x0, hX, hXT, h, H, degree, options );


pval = scaling * out.pval;
disp(['LMI ' int2str(degree) ' lower bound = ' num2str(pval)]);

%% Plot
if ~options.withInputs
    return;
end

figure;
hold on;
% trajectory from simulation
controller = @(tt,xx) [ double(subs(out.u{1},[t;x],[tt;xx])); double(subs(out.u{2},[t;x],[tt;xx])) ];
[ tval, xval ] = ode45( @(tt,xx) scaling*DubinsEq( tt, xx, controller ), [0,out.pval], x0 );
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',4);
% optimal trajectory
V = 1;
omega = 3;
r = V / omega;
p0 = [ x0(1); x0(2)-r ];
pT = [ xT(1); xT(2)+r ];
alpha = atan2( pT(1)-p0(1), pT(2)-p0(2) );
theta = alpha - acos( 2*r/ norm(pT-p0) );
xval1 = p0(1) + r * cos( pi/2:-0.01:theta );
yval1 = p0(2) + r * sin( pi/2:-0.01:theta );
xval2 = pT(1) - r * cos( theta:0.01:pi/2 );
yval2 = pT(2) - r * sin( theta:0.01:pi/2 );
h_traj2 = plot( [xval1, xval2], [yval1, yval2], 'k' );
% auxiliary line
xval3 = r * cos( 0:0.01:2*pi );
yval3 = r * sin( 0:0.01:2*pi );
plot(xval3+p0(1), yval3+p0(2), 'k--');
plot(xval3+pT(1), yval3+pT(2), 'k--');

% legend([h_traj, h_traj2], {'Our trajectory','Optimal trajectory'});
xlim([-1,1]);
ylim([-1,1]);
box on;
xlabel('$x_1$','Interpreter','LaTex','FontSize',20);
ylabel('$x_2$','Interpreter','LaTex','FontSize',20);


% control action
figure;
uval = zeros( length(tval), 2 );
for i = 1 : length(tval)
    tt = tval(i);
    xx = xval(i,1:3)';
    uval(i,1) = double( subs(out.u{1}, [t;x], [tt;xx]) );
    uval(i,2) = double( subs(out.u{2}, [t;x], [tt;xx]) );
end
tval = scaling * tval;
subplot(1,2,1);
hold on;
plot(tval,uval(:,1),'LineWidth',4);
plot([0,tval(end)],[1,1],'k');
ylim([0,1.1]);
xlabel('$t$','Interpreter','LaTex','FontSize',20);
ylabel('$V(t)$','Interpreter','LaTex','FontSize',20);
% hl1 = legend('Our control $V$','Optimal $V$');
% set(hl1,'Interpreter','latex')
subplot(1,2,2);
hold on;
plot(tval,uval(:,2),'LineWidth',4);
plot([0, theta/omega, theta/omega, tval(end)-theta/omega, tval(end)-theta/omega, tval(end)], [-1,-1,0,0,1,1],'k');
% hl2 = legend('Our control $\omega$', 'Optimal $\omega$');
% set(hl2,'Interpreter','latex');
xlabel('$t$','Interpreter','LaTex','FontSize',20);
ylabel('$\omega(t)$','Interpreter','LaTex','FontSize',20);