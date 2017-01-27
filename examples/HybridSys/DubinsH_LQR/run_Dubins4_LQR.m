% Dubins car: 2 modes
% State variables: [x, y, theta]'
% Input: [ V, u ]'
% System dynamics:
% xdot = [ V*cos(1.5*theta)
%          V*sin(1.5*theta)
%          10 * u ]
% 1 | 2
% --|--
% 3 | 4
% 

clear;
T = 3;
d = 6;
nmodes = 4;

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
hU = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );

x0{1} = [ -0.8; 0.8; 0 ];
xT = [ 0.5; -0.2; 0 ];

% Dynamics
% Mode 1
x{1} = xa;
u{1} = ua;
f{1} = T * [ 0;
             0;
             0 ];
g{1} = T * [ polycos(xa(3)), 0;
             polysin(xa(3)), 0;
             0,              3 ];
% Mode 2
x{2} = x{1};
u{2} = u{1};
f{2} = f{1};
g{2} = g{1};
% Mode 3
x{3} = x{1};
u{3} = u{1};
f{3} = f{1};
g{3} = g{1};
% Mode 4
x{4} = x{1};
u{4} = u{1};
f{4} = f{1};
g{4} = g{1};

% Domains
% Mode 1
y = xa;
hX{1} = [ -y(1)*(y(1)+1);
          y(2) * (1-y(2));
          (pi/2)^2 - y(3)^2 ];
hU{1} = [ ua(1) * (1 - ua(1));
          1 - ua(2)^2 ];
hXT{1} = hX{1};

% Mode 2
hX{2} = [ y(1) * (1-y(1));
          y(2) * (1-y(2));
          (pi/2)^2 - y(3)^2 ];
hU{2} = [ ua(1) * (1 - ua(1));
          1 - ua(2)^2 ];
hXT{2} = hX{2};

% Mode 3
hX{3} = [ -y(1)*(y(1)+1);
          -y(2)*(y(2)+1);
          (pi/2)^2 - y(3)^2 ];
hU{3} = [ ua(1) * (1 - ua(1));
          1 - ua(2)^2 ];
hXT{3} = hX{3};

% Mode 4
hX{4} = [ y(1) * (1-y(1));
          -y(2)*(y(2)+1);
          (pi/2)^2 - y(3)^2 ];
hU{4} = [ ua(1) * (1 - ua(1));
          1 - ua(2)^2 ];
hXT{4} = hX{4};

% Guards
% R_12 
sX{1,2} = [ y(1);
            -y(1);
            y(2) * (1-y(2));
            (pi/2)^2 - y(3)^2 ];
R{1,2} = x{2};

% R_13
sX{1,3} = [ -y(1)*(y(1)+1);
            y(2);
            -y(2);
            (pi/2)^2 - y(3)^2 ];
R{1,3} = x{3};

% R_24
sX{2,4} = [ y(1)*(1-y(1));
            y(2);
            -y(2);
            (pi/2)^2 - y(3)^2 ];
R{2,4} = x{4};

% R_34
sX{3,4} = [ y(1);
            -y(1);
            -y(2)*(y(2)+1);
            (pi/2)^2 - y(3)^2 ];
R{3,4} = x{4};

h{1} = (y(1) - xT(1))^2 + (y(2) - xT(2))^2 + ua(1)^2 + ua(2)^2;
h{2} = (y(1) - xT(1))^2 + (y(2) - xT(2))^2 + ua(1)^2 + ua(2)^2;
h{3} = (y(1) - xT(1))^2 + (y(2) - xT(2))^2 + ua(1)^2 + ua(2)^2;
h{4} = (y(1) - xT(1))^2 + (y(2) - xT(2))^2 + ua(1)^2 + ua(2)^2;
H{1} = 0;
H{2} = 0;
H{3} = 0;
H{4} = 0;

% Options
options.MinimumTime = 0;
options.withInputs = 1;
options.svd_eps = 1e5;

% Solve
[out] = HybridOCPDualSolver(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,options);


pval = T * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
% trajectory
figure;
if options.withInputs
    J = @(xx,uu) (xx(1)-0.5)^2 + (xx(2)+0.2)^2 + uu(1)^2 + uu(2)^2;
    ode_options = odeset('Events',@EventFcn);
    [tval,xval] = ode45( @(tt,xx) T*Dubins_4MEq( tt, xx, out.u, J, [t;xa] ), ...
                         [0,1], [x0{1};0], ode_options );
    plot(xval(:,1), xval(:,2));
%     axis equal
    xlim([-1,1]);
    ylim([-1,1]);
    hold on;
    plot(-0.8,0.8,'ro');
    plot(0.5,-0.2,'rx');
end 

% u
uval = zeros( length(tval), 2 );
for i = 1 : length(tval)
    tt = tval(i);
    xx = xval(i,1:3)';
    if ( xx(1) <= 0 ) && ( xx(2) >= 0 )
        uval(i,1) = double( subs(out.u{1,1}, [t;xa], [tt;xx]) );
        uval(i,2) = double( subs(out.u{1,2}, [t;xa], [tt;xx]) );
    elseif ( xx(1) >= 0 ) && ( xx(2) >= 0 )
        uval(i,1) = double( subs(out.u{2,1}, [t;xa], [tt;xx]) );
        uval(i,2) = double( subs(out.u{2,2}, [t;xa], [tt;xx]) );
    elseif ( xx(1) <= 0 ) && ( xx(2) <= 0 )
        uval(i,1) = double( subs(out.u{3,1}, [t;xa], [tt;xx]) );
        uval(i,2) = double( subs(out.u{3,2}, [t;xa], [tt;xx]) );
    else
        uval(i,1) = double( subs(out.u{4,1}, [t;xa], [tt;xx]) );
        uval(i,2) = double( subs(out.u{4,2}, [t;xa], [tt;xx]) );
    end
end
% Saturate u
uval(uval<-1) = -1;
uval(uval>1) = 1;

figure;
subplot(1,2,1);
plot(tval,uval(:,1));
ylim([0,1.1]);
subplot(1,2,2);
plot(tval,uval(:,2));

