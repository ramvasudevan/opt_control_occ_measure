% Double Integrator - Minimum Time, using hybrid version of OCP code
% Target set : {(0,0)}

clear;
scaling = 3;
d = 6;
nmodes = 1;

% Define variables
t = msspoly( 't', 1 );
xa = msspoly( 'x', 2 );
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

x0{1} = [ 0.3; 1 ];

% Dynamics
x{1} = xa;
u{1} = ua;
f{1} = scaling * [ xa(2); 0 ];
g{1} = scaling * [ 0; 1 ];

% Domains
y = xa;
hX{1} = y(2) + 1;
hXT{1} = -y.^2;
h{1} = 1;
H{1} = 0;

% Options
options.MinimumTime = 1;
options.withInputs = 1;

% Solve
[out] = HybridOptimalControlDualSolver(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);

pval = scaling * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% plot
x0 = x0{1};
t1 = x0(2) + sqrt(x0(1) + x0(2)^2/2);
t2 = sqrt(x0(1) + x0(2)^2/2);

% Trajectories
figure;
hold on;

% Trajectory from simulation
controller = @(tt,xx) double(subs(out.u{1},[t;xa],[tt;xx]));
[ tval, xval ] = ode45( @(tt,xx) scaling*DIEq( tt, xx, controller ), [0:0.01:pval/scaling], x0 );
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
controller = @(tt,xx) double(subs(out.u{1},[t;xa],[tt;xx]));
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