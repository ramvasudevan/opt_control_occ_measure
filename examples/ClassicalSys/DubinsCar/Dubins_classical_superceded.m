% Double integrator - Minimum Time

clear;
T = 3;
degree = 6;

% polysin = @(x) x - x^3/6;
% polycos = @(x) 1 - x^2/2 + x^4/24;
polysin = @(x) x;
polycos = @(x) 1 - x^2/2;

% dynamics
t = msspoly( 't', 1 );
x = msspoly( 'x', 3 );
u = msspoly( 'u', 2 );
f = [ 0; 0; 0 ];
g = [ polycos(1.5*x(3)), 0;
      polysin(1.5*x(3)), 0;
      0,                 2 ];

x0 = [ -0.8; 0.4; 0 ];
xT = [ 0.5; -0.2; 0 ];
hX = 1 - x.^2;
hXT = [ - (x(1:2)-xT(1:2)).^2;
        1 - x(3)^2 ];
hU = 1 - u.^2;
h = 1;
H = 0;

% options
opts.freeT = 1;
opts.noinput = 0;

% Solve
[out] = OCP_Controller_Dual( t, x, u, f, g, x0, hX, hXT, hU, T, h, H, degree, opts );
disp( out.pval );

%% Plot

% Trajectories
figure;
hold on;
% trajectory from simulation
controller = @(tt,xx) [ double(subs(out.u{1},[t;x],[tt;xx])); double(subs(out.u{2},[t;x],[tt;xx])) ];
[ tval, xval ] = ode45( @(tt,xx) T*DubinsEq( tt, xx, controller ), [0,out.pval/T], x0 );
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',2);


% % optimal trajectory
% V = 1;
% omega = 3;
% r = V / omega;
% p0 = [ x0(1); x0(2)-r ];
% pT = [ xT(1); xT(2)+r ];
% alpha = atan2( pT(1)-p0(1), pT(2)-p0(2) );
% theta = alpha - acos( 2*r/ norm(pT-p0) );
% xval1 = p0(1) + r * cos( pi/2:-0.01:theta );
% yval1 = p0(2) + r * sin( pi/2:-0.01:theta );
% xval2 = pT(1) - r * cos( theta:0.01:pi/2 );
% yval2 = pT(2) - r * sin( theta:0.01:pi/2 );
% h_traj2 = plot( [xval1, xval2], [yval1, yval2], 'k', 'LineWidth', 2 );
% % auxiliary line
% xval3 = r * cos( 0:0.01:2*pi );
% yval3 = r * sin( 0:0.01:2*pi );
% plot(xval3+x0(1), yval3+p0(2), 'k--');
% plot(xval3+xT(1), yval3+pT(2), 'k--');
% 
% legend([h_traj, h_traj2], {'Our trajectory','Optimal trajectory'});
% xlim([-1,1]);
% ylim([-1,1]);

% Control action
