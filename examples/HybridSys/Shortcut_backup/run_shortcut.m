% Shortcut problem: 3 modes, 2 inputs
% It's different from Shortcut in the way that R12 is now shifted and scaled.
% With more interesting reset map: -x^2 + 0.5
% Mode 1 and Mode 2: Dubins Car
% xdot = [ V*cos(theta)
%          V*sin(theta)
%          3 * omega ]
% V \in [0,1], omega \in [-3,3]
% 
% Mode 3: Shortcut
% xdot = 10 * V
% 
% -------
% |  1  |    |
% |-----|    | <- 3
% |  2  |    |
% -------
% 

clear;
T = 3;
d = 10;
nmodes = 3;

polysin = @(x) x;
polycos = @(x) 1 - x^2/2;

% Define variables
t = msspoly( 't', 1 );
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

% Dynamics
% Mode 1 (x,y,theta)
x{1} = msspoly( 'xa', 3 );
u{1} = ua;
y = x{1};
f{1} = T * [ 0;
             0;
             0 ];
g{1} = T * [ polycos(y(3)), 0;
             polysin(y(3)), 0;
             0,             3 ];
% Mode 2 (x,y,pi+theta)
x{2} = x{1};
u{2} = u{1};
f{2} = f{1};
g{2} = g{1};
               
% Mode 3
x{3} = msspoly( 'xb', 1 );
u{3} = ua;
f{3} = T * 0;
g{3} = T * [ -2, 0 ];

% Domains
% -------------- Mode 1 ---------------
y = x{1};
hX{1} = [ 1 - y(1).^2;
          y(2) * (1-y(2));
          (pi/2)^2 - y(3).^2 ];
hU{1} = [ ua(1) * (1 - ua(1));
          1 - ua(2)^2 ];
% hU{1} = [ -(1 - ua(1))^2;
%           1 - ua(2)^2 ];
% R_13
sX{1,3} = [ y(2) - 1;
            1 - y(2);
            hX{1} ];

R{1,3} = -y(1);
% R_12
sX{1,2} = [ -y(2);
            hX{1} ];
R{1,2} = y;
% cost function
h{1} = 1;
% h{1} = -ua(1);
H{1} = 0;

% -------------- Mode 2 ----------------
y = x{2};
hX{2} = [ 1 - y(1).^2;
          -y(2) * (y(2)+1);
          (pi/2)^2 - y(3).^2 ];
hU{2} = [ ua(1) * (1 - ua(1));
          1 - ua(2)^2 ];
% hU{2} = [ -(1 - ua(1))^2;
%           1 - ua(2)^2 ];
hXT{2} = [ - (y(1) - 0.8)^2;
           - (y(2) + 0.8)^2;
           (pi/2)^2 - y(3)^2 ];
% cost function
h{2} = 1;
% h{2} = -ua(1);
H{2} = 0;

% -------------- Mode 3 ----------------
y = x{3};
hX{3} = 1 - y.^2;
hU{3} = ua(1) * (1 - ua(1));
% hU{3} = [ -(1 - ua(1))^2;
%           1 - ua(2)^2 ];
% R_32
sX{3,2} = - (y+1)^2;
R{3,2} = [ 0.6; -0.8; 0 ];
% cost function
h{3} = 1;
% h{3} = -ua(1);
H{3} = 0;

% options
options.MinimumTime = 1;
options.withInputs = 1;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);

pval = T * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
if ~options.withInputs
    return;
end

figure;
hold on;
% trajectory from simulation
% In mode 1
controller1 = @(tt,xx) [ double(subs(out.u{1,1},[t;x{1}],[tt;xx])); double(subs(out.u{1,2},[t;x{1}],[tt;xx])) ];
ode_options = odeset('Events', @EventFcn_1);
[ tval1, xval1 ] = ode45( @(tt,xx) T*DubinsEq( tt, xx, controller1 ), [0:0.0001:1], x0{1}, ode_options );
h_traj1 = plot3( xval1(:,1), xval1(:,2), xval1(:,1)*0, 'LineWidth', 4 );

% In mode 3
ode_options = odeset('Events', @EventFcn_3);
[ tval3, xval3 ] = ode45( @(tt,xx) -2*T*double(subs(out.u{3,1},[t;x{3}],[tt;xx])), ...
                          [tval1(end):0.0001:1], -xval1(end,1), ode_options );
h_traj3 = plot3( xval3*0+1, xval3, xval3*0+1, 'LineWidth', 4 );

% In mode 2
controller2 = @(tt,xx) [ double(subs(out.u{2,1},[t;x{1}],[tt;xx])); double(subs(out.u{2,2},[t;x{2}],[tt;xx])) ];
ode_options = odeset('Events', @EventFcn_2);
[ tval2, xval2 ] = ode45( @(tt,xx) T*DubinsEq( tt, xx, controller2 ), ...
                          [tval3(end):0.0001:1], [ 0.6; -0.8; 0 ], ode_options );
h_traj2 = plot3( xval2(:,1), xval2(:,2), xval2(:,1)*0, 'LineWidth', 4 );

% control action
figure;
hold on;
uval1 = zeros( length(tval1), 2 );
for i = 1 : length(tval1)
    tt = tval1(i);
    xx = xval1(i,1:3)';
    uval1(i,1) = double( subs(out.u{1,1}, [t;x{1}], [tt;xx]) );
    uval1(i,2) = double( subs(out.u{1,2}, [t;x{1}], [tt;xx]) );
end
uval2 = zeros( length(tval2), 1 );
for i = 1 : length(tval2)
    tt = tval2(i);
    xx = xval2(i,:)';
    uval2(i,1) = double( subs(out.u{2,1}, [t;x{2}], [tt;xx]) );
    uval2(i,2) = double( subs(out.u{2,2}, [t;x{2}], [tt;xx]) );
end
uval3 = zeros( length(tval3), 1 );
for i = 1 : length(tval3)
    tt = tval3(i);
    xx = xval3(i);
    uval3(i) = double( subs(out.u{3,1}, [t;x{3}], [tt;xx]) );
end

uval1(uval1>1) = 1;
uval2(uval2>1) = 1;
uval3(uval3>1) = 1;

subplot(1,2,1);
plot([tval1;tval3;tval2],[uval1(:,1);uval3(:,1);uval2(:,1)]);
subplot(1,2,2);
plot([tval1;tval3;tval2],[uval1(:,2);uval3(:,1)*nan;uval2(:,2)]*3);
