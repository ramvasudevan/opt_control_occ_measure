% Double integrator - Minimum Time
% Target set : {(0,0)}

clear;
scaling = 3;
degree = 6;

% dynamics
t = msspoly( 't', 1 );
x = msspoly( 'x', 2 );
u = msspoly( 'u', 1 );
f = scaling * [ x(2); 0 ];
g = scaling * [ 0; 1 ];

x0 = [ 0.3; 1 ];
hX = x(2) + 1;
hXT = -x.^2;
h = 1;
H = 0;

% options
opts.MinimumTime = 1;
opts.withInputs = 1;

% Solve
[out] = OCP_Controller_Dual( t, x, u, f, g, x0, hX, hXT, h, H, degree, opts );
disp( out.pval );

%% plot
t1 = x0(2) + sqrt(x0(1) + x0(2)^2/2);
t2 = sqrt(x0(1) + x0(2)^2/2);

% Trajectories
figure;
hold on;

% Trajectory from simulation
controller = @(tt,xx) double(subs(out.u{1},[t;x],[tt;xx]));
[ tval, xval ] = ode45( @(tt,xx) scaling*DIEq( tt, xx, controller ), [0:0.01:2.79./scaling], x0 );
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',4);
% Optimal trajectory
controller = @(tt,xx) 1-2*(tt <= t1);
[ ~, xvalA1 ] = ode45( @(tt,xx) DIEq( tt, xx, controller ), ...
                       [0:0.001:t1], x0 );
[ ~, xvalA2 ] = ode45( @(tt,xx) DIEq( tt, xx, controller ), ...
                       [t1:0.001:(t1+t2)],...
                       xvalA1(end,:) );
xvalA1 = [xvalA1; xvalA2];
h_traj2 = plot(xvalA1(:,1), xvalA1(:,2), 'k');
plot(x0(1),x0(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot(0,0,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
xlim([-1,1]);
ylim([-1,1]);
xlabel('$x_1$','Interpreter','LaTex','FontSize',20);
ylabel('$x_2$','Interpreter','LaTex','FontSize',20);
box on;
% legend([h_traj, h_traj2], {'Our controller','Optimal controller'});

% Control action
figure;
hold on;

% Simulation
controller = @(tt,xx) double(subs(out.u{1},[t;x],[tt;xx]));
uval = zeros( size(tval) );
for i = 1 : length(tval)
    uval(i) = controller( tval(i), xval(i,:)' );
end
h_u = plot(tval*scaling, uval, 'LineWidth', 4);
% Optimal
h_u2 = plot([0,t1,t1,t1+t2],[-1,-1,1,1],'k');
% legend([h_u, h_u2], {'Our controller', 'Optimal controller'});
xlabel('$t$','Interpreter','LaTex','FontSize',20);
ylabel('$u(t)$','Interpreter','LaTex','FontSize',20);