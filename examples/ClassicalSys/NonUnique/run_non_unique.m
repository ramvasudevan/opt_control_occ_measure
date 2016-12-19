% close all;
clear;
scaling = 3;
degree = 10;

% dynamics
t = msspoly( 't', 1 );
x = msspoly( 'x', 2 );
u = msspoly( 'u', 1 );
f = scaling * [ 1; 0 ];
g = scaling * [ 0; 1 ];

x0 = [ -1; 0 ];
hX = [ 1 - x(1)^2;
       1 - x(2)^2 ];
hXT = [ -(x(1)-1)^2;
        - x(2)^2 ];
h = -x(2)^2;
H = 0;

% options
options.freeFinalTime = 1;
options.withInputs = 1;

% Solve
[ out ] = OCP_Controller_Dual( t, x, u, f, g, x0, hX, hXT, h, H, degree, options );

pval = scaling * out.pval;
disp(['LMI ' int2str(degree) ' lower bound = ' num2str(pval)]);

%% Plot
figure(1);
hold on;


controller = @(tt,xx) double(subs(out.u{1,1},[t;x],[tt;xx]));
dyn = @(tt,xx) scaling * [ 1; controller(tt,xx) ];

x0p = [-0.8, 0.2];
t0p = 0.2 / scaling;

[ tval, xval ] = ode45(@(tt,xx) dyn(tt,xx), [t0p:0.01:0.5], [x0p] );

plot(xval(:,1), xval(:,2));
xlim([-1,1]);
ylim([-1,1]);