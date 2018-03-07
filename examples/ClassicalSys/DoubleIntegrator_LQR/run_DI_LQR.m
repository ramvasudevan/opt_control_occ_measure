% Double integrator with 2 modes - minimum time problem
% xdot = [ x_2 ] + [ 0 ] * u
%        [ 0   ]   [ 1 ]
% 
% u(t) \in [-1, 1]
% X_1 = { x | x_1^2 + x_2^2 <= r2 }
% X_2 = { x | x_1^2 + x_2^2 >= r2 }
% XT_1 = {(0,0)}
% XT_2 = {empty}
% h = 1, H = 0
% 
% Trajectory starts at (1,1) in mode 2, and minimizes the running cost
% h = x'*x + 20 * u^2 up to time T
% 

clear;
T = 5;         % time horizon
d = 12;          % degree of relaxation
nmodes = 1;     % number of modes

% Define variables
t = msspoly( 't', 1 );
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

x0{1} = [ 1; 1 ];

% Dynamics
x{1} = msspoly( 'x', 2 );
u{1} = msspoly( 'u', 1 );
f{1} = T * [ x{1}(2); 0 ];
g{1} = T * [ 0; 1 ];

% Domains
% Mode 1 
y = x{1};
hX{1} = y(2)+1;
hU{1} = 1 - u{1}^2;
hXT{1} = hX{1};
h{1} = 1*x{1}' * x{1} + 20 * u{1}^2;
H{1} = 0;

% Options
options.MinimumTime = 0;
options.withInputs = 1;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);

pval = T * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
xs0 = x0{1};
figure;
hold on;
box on;

% Integrate forward trajectory
controller = [ out.u{1} ];
J = @(xx, uu) xx'*xx + 20 * uu^2;
[ tval, xval ] = ode45(@(tt,xx) T * DIEq( tt, xx, controller, J, [t;x{1}] ), ...
                       [0:0.01:1], [xs0; 0] );
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',2);

plot(x0{1}(1),x0{1}(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410]);
plot(0,0,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410]);
xlim([-1,2]);
ylim([-1,1]);
set(gca,'XTick',[-1,2]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
box on;

% Control
figure;
hold on;
uval = zeros( size(tval) );
for i = 1 : length(tval)
        uval(i) = double(subs(controller(1), [t;x{1}], [tval(i);xval(i,1:2)']));
end
plot(tval*T, uval,'Linewidth',2);

xlim([0,T]);