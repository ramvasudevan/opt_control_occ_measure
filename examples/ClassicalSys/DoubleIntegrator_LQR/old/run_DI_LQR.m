% Double Integrator - LQR problem, using classical version of OCP code
close all;
clear;
scaling = 5;
degree = 12;

% dynamics
t = msspoly( 't', 1 );
x = msspoly( 'x', 2 );
u = msspoly( 'u', 1 );
f = scaling * [ x(2); 0 ];
g = scaling * [ 0; 1 ];

x0 = [ 1; 1 ];
hX = x(2) + 1;
hXT = x(2) + 1;
h = x(1)^2 + x(2)^2 + 20*u^2;
H = 0;

% options
options.freeFinalTime = 0;
options.withInputs = 1;
options.svd_eps = 1e3;

% Solve
[ out ] = OCPDualSolver( t, x, u, f, g, x0, hX, hXT, h, H, degree, options );

pval = scaling * out.pval;
disp(['LMI ' int2str(degree) ' lower bound = ' num2str(pval)]);

%% plot

% Trajectories
figure(1);
hold on;

% Trajectory from simulation
controller = @(tt,xx) double(subs(out.u{1,1},[t;x],[tt;xx]));
[ tval, xval ] = ode45( @(tt,xx) scaling*DIEq( tt, xx, controller ), [0:0.01:1], [x0; 0] );
cost = xval(end,end);
xval = xval(:,1:2);
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',4);
plot(x0(1),x0(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot(0,0,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
xlabel('$x_1$','Interpreter','LaTex','FontSize',20);
ylabel('$x_2$','Interpreter','LaTex','FontSize',20);
box on;
% axis equal;
% legend([h_traj, h_traj2], {'Our controller','Optimal controller'});

% Control action
figure(2);
hold on;

% Simulation
xval = xval(:,1:2);
controller = @(tt,xx) double(subs(out.u{1},[t;x],[tt;xx]));
uval = zeros( size(tval) );
for i = 1 : length(tval)
    uval(i) = controller( tval(i), xval(i,:)' );
end
h_u = plot(tval*scaling, uval, 'LineWidth', 4);
xlabel('$t$','Interpreter','LaTex','FontSize',20);
ylabel('$u(t)$','Interpreter','LaTex','FontSize',20);

standardLQR;