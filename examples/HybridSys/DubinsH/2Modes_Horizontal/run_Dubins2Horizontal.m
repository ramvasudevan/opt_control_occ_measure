% Dubins car: 2 modes, horizontal
% State variables: [x, y, theta]'
% Input: [ V, u ]'
% System dynamics:
% xdot = [ V*cos(1.5*theta)
%          V*sin(1.5*theta)
%          2 * u ]
%  1
% ---
%  2

clear;
scaling = 3;
d = 10;
nmodes = 2;

polysin = @(x) x;
polycos = @(x) 1 - x^2/2;

% Define variables
t = msspoly( 't', 1 );
xa = msspoly( 'x', 3 );
ua = msspoly( 'u', 2 );
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

x0{1} = [ -0.8; 0.8; 0 ];

% Dynamics
% Mode 1
x{1} = xa;
u{1} = ua;
f{1} = scaling * [ 0;
                   0;
                   0 ];
g{1} = scaling * [ polycos(1.5*xa(3)), 0;
                   polysin(1.5*xa(3)), 0;
                   0,                  2 ];
% Mode 2
x{2} = x{1};
u{2} = u{1};
f{2} = f{1};
g{2} = g{1};

% Domains
% Mode 1
y = x{1};
hX{1} = [ 1 - y(1).^2;
          y(2) * (1-y(2));
          1 - y(3).^2 ];
sX{1,2} = [ 1 - y(1)^2;
            - y(2)^2;
            1 - y(3)^2 ];
R{1,2} = x{2};
h{1} = 1;
% Mode 2
y = x{2};
hX{2} = [ 1 - y(1).^2;
          -y(2) * (y(2)+1);
          1 - y(3).^2 ];
hXT{2} = [ - (y(1) - 0.8)^2;
           - (y(2) + 0.8)^2;
           1 - y(3)^2 ];
h{2} = 1;
H{2} = 0;

% Options
options.MinimumTime = 1;
options.withInputs = 1;
options.svd_eps = 1e4;

% Solve
[out] = HybridOptimalControlDualSolver1(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);
% [out] = HybridOptimalControlDualSolver(t,x,u,f,g,hX,sX,R,x0,hXT,h,H,d,options);


pval = scaling * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
if ~options.withInputs
    return;
end

% trajectory
J = @(x) 1;
% ode_options = odeset('Events',@EventFcn);
[tval,xval] = ode45( @(tt,xx) scaling*Dubins_2MEq_Horizontal( tt, xx, out.u, J, [t;xa] ), ...
                     [0,out.pval], [x0{1};0] );
plot(xval(:,1), xval(:,2));
hold on;
plot(-0.8,0.8,'ro');
plot(0.8,-0.8,'rx');

% u
uval = zeros( length(tval), 2 );
for i = 1 : length(tval)
    tt = tval(i);
    xx = xval(i,1:3)';
    if xx(2) >= 0
        uval(i,1) = double( subs(out.u{1,1}, [t;xa], [tt;xx]) );
        uval(i,2) = double( subs(out.u{1,2}, [t;xa], [tt;xx]) );
    else
        uval(i,1) = double( subs(out.u{2,1}, [t;xa], [tt;xx]) );
        uval(i,2) = double( subs(out.u{2,2}, [t;xa], [tt;xx]) );
    end
end
figure;
subplot(1,2,1);
plot(tval,uval(:,1));
subplot(1,2,2);
plot(tval,uval(:,2));