% Double integrator with 2 modes - Minimum Time
% Goal: reach point (0,0) in minimum time
% running cost = 1
% terminal cost = 0
% 
% Mode 1: x^2+y^2 < r2
% Mode 2: x^2+y^2 > r2
% 

clear;
T = 15;         % time horizon
d = 6;          % degree of relaxation
nmodes = 2;     % number of modes
r2 = 0.3;       % r^2, where r is the radius of the domain of mode 1

% Define variables
t = msspoly( 't', 1 );
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

x0{2} = [ 1; 1 ];

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
hXT{1} = hX{1};
h{1} = x{1}' * x{1} + 20 * u{1}^2;
H{1} = 0;

% Mode 2
y = x{2};
hX{2} = [ y'*y - r2;
          y(2) + 1 ];
hXT{2} = hX{2};
sX{2,1} = [ r2 - y' * y;
            y' * y - r2 ];
R{2,1} = x{1};
h{2} = x{2}' * x{2} + 20 * u{2}^2;
H{2} = 0;

% Options
options.MinimumTime = 0;
options.withInputs = 1;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCPDualSolver(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);

pval = T * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
xs0 = x0{2};
% trajectory
figure;
hold on;
box on;

circx = -sqrt(r2):0.001:sqrt(r2);
circy = sqrt( r2 - circx .^ 2 );
h_area2 = area([-2;3],[-2,4;-2,4],'ShowBaseLine','off','FaceColor',[0.6 1 0.6]);
h_area = area(circx', [-circy; 2*circy]', 'BaseValue',0, 'ShowBaseLine','off','FaceColor',[1 0.6 0.6]);

controller = [ out.u{1}; out.u{2} ];
% ode_options = odeset('Events',@EventFcn);
% ode_options = odeset;
% Trajectory from simulation
[ tval, xval ] = ode45(@(tt,xx) T * Hybrid_DIEq( tt, xx, controller, @(x) 1, [t;x{1}] ), ...
                       [0:0.001:1], [xs0; 0] );
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',2);
% Actual optimal trajectory
% [ ~, xvalA1 ] = ode45(@(tt,xx) DIEq_optimal( tt, xx, xs0 ), [0:0.001:xs0(2) + sqrt( xs0(1) + xs0(2)^2/2 )], xs0 );
% [ ~, xvalA2 ] = ode45(@(tt,xx) DIEq_optimal( tt, xx, xs0 ), [xs0(2) + sqrt( xs0(1) + xs0(2)^2/2 ):0.001: 0.6*scaling], xvalA1(end,:));
% xvalA1 = [xvalA1;xvalA2];
% h_traj2 = plot(xvalA1(:,1), xvalA1(:,2), 'k--');

% plot([-1,1.5],[-1,-1],'k');
plot(x0{2}(1),x0{2}(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410]);
plot(0,0,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410]);
xlim([-1,2]);
ylim([-1,1]);
set(gca,'XTick',[-1,2]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
box on;
% legend([h_area(2) h_area2(2) h_traj h_traj2], {'Mode 1', 'Mode 2', 'Trajectory under extracted control law', 'Optimal trajectory'});

% [~,idx] = find( id_event == 2 );
% tcross = scaling * time_event( idx(1) );

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
plot(tval*5, uval,'Linewidth',2);
% tvalu = [0, xs0(2) + sqrt( xs0(1) + xs0(2)^2/2 ), xs0(2) + sqrt( xs0(1) + xs0(2)^2/2 ), 0.75*scaling];
% uvalu = [-1,-1,1,1];
% plot(tvalu, uvalu,'--k');
% legend('Extracted control','Optimal value');

xlim([0,3.5]);