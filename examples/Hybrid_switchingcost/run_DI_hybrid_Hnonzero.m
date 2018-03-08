% Double integrator - minimum time problem
% xdot = [ x_2 ] + [ 0 ] * u
%        [ 0   ]   [ 1 ]
% 
% u(t) \in [-1, 1]
% X_1 = { x | x_1^2 + x_2^2 <= r2 }
% X_2 = { x | x_1^2 + x_2^2 >= r2 }
% XT_1 = X_1
% XT_2 = X_2
% h = 0, H = x.^2
% 
% Trajectory starts at (0.3,1) in mode 2, and get close to (0,0) at T
% 


clear;
T = 3;
d = 6;
nmodes = 2;
r2 = 0.3;

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

x0{2} = [ 0.3; 1 ];

% Dynamics
x{1} = msspoly( 'x', 2 );
u{1} = msspoly( 'u', 1 );
f{1} = T * [ x{1}(2); 0 ];
g{1} = T * [ 0; 1 ];

x{2} = x{1};
u{2} = u{1};
f{2} = f{1};
g{2} = g{1};

% Domains
% Mode 1 
y = x{1};
hX{1} = r2 - y'*y;
hU{1} = 1 - u{1}.^2;
hXT{1} = hX{1};
h{1} = 0;
H{1} = y' * y;
% H{1} = y(1);

% Mode 2
y = x{2};
hX{2} = [ y'*y - r2;
          y(2) + 1 ];
hU{2} = 1 - u{2}.^2;
hXT{2} = hX{2};
sX{2,1} = [ r2 - y' * y;
            y' * y - r2 ];
R{2,1} = x{1};
h{2} = 0;
H{2} = y' * y;
% H{2} = y(1);

% Options
options.MinimumTime = 0;
options.withInputs = 1;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCPDualSolver1(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,[],d,options);

pval = T * out.pval;

disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
% trajectory
figure;
hold on;
box on;

circx = -sqrt(r2):0.001:sqrt(r2);
circy = sqrt( r2 - circx .^ 2 );
h_area2 = area([-2;3],[-2,4;-2,4],'ShowBaseLine','off','FaceColor',[0.6 1 0.6]);
h_area = area(circx', [-circy; 2*circy]', 'BaseValue',0, 'ShowBaseLine','off','FaceColor',[1 0.6 0.6]);

xs0 = x0{2};
controller = [ out.u{1}; out.u{2} ];
ode_options = odeset('Events',@EventFcn);
% Trajectory from simulation
[ tval, xval, time_event, ~, id_event ] = ode45(@(tt,xx) T * Hybrid_DIEq( tt, xx, controller, @(x) 1, [t;x{1}] ), ...
                       [0:0.01:1], [xs0; 0], ode_options );
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',2);

plot(x0{2}(1),x0{2}(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410]);
plot(0,0,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410]);
xlim([-1,1]);
ylim([-1,1]);
set(gca,'XTick',[-1,1]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
box on;

% u
figure;
hold on;
uval = zeros( size(tval) );
for i = 1 : length(tval)
    if (xval(i,1:2)'*xval(i,1:2) <= r2)
        uval(i) = double(subs(controller(1), [t;x{1}], [tval(i);xval(i,1:2)']));
    else
        uval(i) = double(subs(controller(2), [t;x{1}], [tval(i);xval(i,1:2)']));
    end
end
plot(tval*T, uval,'Linewidth',2);
xlim([0,T]);