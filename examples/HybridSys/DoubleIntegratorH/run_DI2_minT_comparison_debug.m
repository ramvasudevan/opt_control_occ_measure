% Run Double Integrator, but only in mode 2
% Trajectory starts at (1,0.3) and evolves until x1^2+x2^2 = 0.3
% Solves using non-hybrid solver

clear;
T = 5;
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

% Domains and transitions
% Mode 1 
y = x{1};
hX{1} = r2 - y'*y;      % { x | x_1^2 + x_2^2 <= r2 }
hU{1} = 1 - u{1}^2;     % [-1, 1]
hXT{1} = 0-y.^2;        % {(0,0)}
h{1} = 1;
H{1} = 0;

% Mode 2
y = x{2};
hX{2} = [ ...
    y'*y - r2;      % { x | x_1^2 + x_2^2 >= r2 }
    1 - y.^2 ];
hX{2} = y'*y - r2;
hU{2} = 1 - u{2}^2;     % [-1, 1]
% sX{2,1} = [ r2 - y' * y;
%             y' * y - r2 ];
% R{2,1} = x{1};          % Identity reset map
h{2} = 1;
H{2} = 0;

% Options
options.freeFinalTime = 1;
options.withInputs = 1;
options.svd_eps = 1e4;

% Solve

hXT{2} = [ ...
    r2 - y' * y;
    y' * y - r2 ];
[out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);
% [out] = HybridOCP_Comparison(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,seq,d,options);

%% Pval
pval = T * out.pval;

% Calculate actual T
xs0 = x0{2};
if xs0(1) >= -(xs0(2)^2-2)/2
    Ta = 1+xs0(1)+xs0(2)+xs0(2)^2/2;
elseif xs0(1) >= -xs0(2)^2/2*sign(xs0(2)) 
    Ta = 2*sqrt(xs0(1)+xs0(2)^2/2)+xs0(2);
else
    Ta = 2*sqrt(-xs0(1)+xs0(2)^2/2)-xs0(2);
end

disp(pval / Ta);
disp(['Minimum time = ' num2str(Ta)]);
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
figure;
hold on;

th = 0:0.01:2*pi;
circx = sqrt(r2) * cos(th);
circy = sqrt(r2) * sin(th);
plot(circx, circy, 'k');

ode_options = odeset('Events',@EventFcn_comparison);

% Integrate forward
% controller = [ out(1).u{1}; out(2).u{1} ];
% [ tval, xval, time_event, ~, id_event ] = ode45(@(tt,xx) T * Hybrid_DIEq( tt, xx, controller, @(x) 1, [t;x{1}] ), ...
%                        [0:0.001:0.75], [xs0; 0], ode_options );
controller = [ out(1).u{2} ];
[ tval, xval, time_event, ~, id_event ] = ode45(@(tt,xx) T * Hybrid_DIEq_comparison( tt, xx, controller, @(x) 1, [t;x{1}] ), ...
                       [0:0.001:0.75], [xs0; 0], ode_options );
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',2);

plot(x0{2}(1),x0{2}(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410]);
plot(0,0,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410]);
% xlim([-1,1]);
% ylim([-1,1]);
set(gca,'XTick',[-1,1]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
box on;


function xdot = Hybrid_DIEq_comparison( t, x, u, J, var )
xval = x(1:2);
uval = double(subs(u(1), var, [t;xval]));
xdot = [ xval(2);
         uval;
         J(xval) ];
end

function [value,isterminal,direction] = EventFcn_comparison(~,y)
    r2 = 0.3;
    value = r2 - y.' * y;
    direction = 0;
    isterminal = 1;
end