% Mode 1 is Dubins Car
% Mode 2 is 1-dimensional dummy mode with zero dynamics
% Trajectory starts at (-0.8,0.8,0) in mode 1, with V = 1 and omega=V/r1
% r2 is the radius of the central 'hole'
% 

clear;
close all;
T = 2;        % maximum time horizon
d = 6;          % degree of relaxation
dr = 3;
nmodes = 2;     % number of hybrid modes

polysin = @(x) x;
polycos = @(x) 1 - x^2/2;

% ============================ Parameters ================================
V = 1;      % Linear velocity
r1 = 2;     % Turning radius
r2 = 0.465;   % Central hole radius

omega = -V / r1;

% ========================= Define variables =============================
t = msspoly( 't', 1 );
x = cell( nmodes, 1 );
f = cell( nmodes, 1 );
g = cell( nmodes, 1 );
x0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );

x0{1} = [ -0.8; 0.7; 0 ];

% =============================== Dynamics ===============================
% -------  Mode 1 --------
x{1} = msspoly( 'xa', 3 );
y = x{1};
u{1} = [V; omega];
f{1} = T * [ 0;
             0;
             0 ];
g{1} = T * [ polycos(y(3)), 0;
             polysin(y(3)), 0;
             0,             1 ];
% -------  Mode 2 --------
x{2} = msspoly( 'xb', 1 );
u{2} = 0;
f{2} = 0;
g{2} = 0;

% ======================== Domains and transitions =======================
% ----------- Mode 1 -----------
y = x{1};
hX{1} = [ 1 - y(1).^2;              % X_1 = [-1,1] x [-1,1] x [-pi/2,pi/2]
          1 - y(2).^2;              % \Union x^2+y^2>=r2^2
          (pi/2)^2 - y(3).^2;
          y(1).^2 + y(2).^2 - r2^2 ];
% transition 1->2
sX{1,2} = [ y(1).^2 + y(2).^2 - r2^2	% S_12 = {x^2+y^2==r2^2}
            -y(1).^2 - y(2).^2 + r2^2;
            (pi/2)^2 - y(3).^2; ];
R{1,2} = 0;                         % R_12 = 0

% -------------- Mode 2 ----------------
y = x{2};
hX{2} = [ 1 - y(1).^2 ];            % X_2 = [-1,1]


% =============================== Solve ==================================
options.withTransition = 1;

[out] = HybridSimulator(t,x,u,f,g,hX,sX,R,x0,d,dr,options);

% ================================ Plot ==================================
dyn = @(tt,xx) [polycos(xx(3)); polysin(xx(3)); omega];
[tval,xval] = ode45(dyn, [0,T], x0{1});

dat = xval(:,1).^2 + xval(:,2).^2 - r2^2;
idx = find(dat < 0, 1, 'first');
disp(['first crossing time = ', num2str(tval(idx))]);

% x_cent = -0.8;
% y_cent = 0.8 - r1;
% theta = pi/2:-0.01:(pi/2+omega*T);
% xval1 = x_cent + r1 * cos(theta);
% yval1 = y_cent + r1 * sin(theta);

xval2 = r2 * cos(0:0.01:2*pi);
yval2 = r2 * sin(0:0.01:2*pi);

figure;
hold on;
axis equal
xlim([-1,1]);
ylim([-1,1]);
box on;
plot(xval(:,1), xval(:,2));
plot(xval2, yval2);

out.trans{1}*T
out.trans{2}*T


