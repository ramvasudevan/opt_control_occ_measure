function plotdata(filename,mythick,mycolor)

load(filename);
disp(['Processing ',filename]);

% Trajectory
figure(1);
J = @(xx) 1;

% In mode 1
controller1 = @(tt,xx) [ double(subs(out.u{1,1},[t;x{1}],[tt;xx])); double(subs(out.u{1,2},[t;x{1}],[tt;xx])) ];
ode_options = odeset('Events', @EventFcn_1);
[ tval1, xval1 ] = ode45( @(tt,xx) T*DubinsEq( tt, xx, controller1 ), [0:0.002:1], x0{1}, ode_options );
h_traj1 = plot3( xval1(:,1), xval1(:,2), xval1(:,1)*0,'LineWidth',mythick,'color',mycolor );

% In mode 3
ode_options = odeset('Events', @EventFcn_3);
[ tval3, xval3 ] = ode45( @(tt,xx) -2*T*double(subs(out.u{3,1},[t;x{3}],[tt;xx])), ...
                          [tval1(end):0.002:1], -xval1(end,1), ode_options );
h_traj3 = plot3( xval3*0+1, xval3, xval3*0+1,'LineWidth',mythick,'color',mycolor );

% In mode 2
controller2 = @(tt,xx) [ double(subs(out.u{2,1},[t;x{1}],[tt;xx])); double(subs(out.u{2,2},[t;x{2}],[tt;xx])) ];
ode_options = odeset('Events', @EventFcn_2);
[ tval2, xval2 ] = ode45( @(tt,xx) T*DubinsEq( tt, xx, controller2 ), ...
                          [tval3(end):0.002:1], [ 0.6; -0.8; 0 ], ode_options );
h_traj2 = plot3( xval2(:,1), xval2(:,2), xval2(:,1)*0,'LineWidth',mythick,'color',mycolor );


% Control
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
uval1(uval1<0) = 0;
uval2(uval2>1) = 1;
uval2(uval2<0) = 0;
uval3(uval3>1) = 1;
uval3(uval3<0) = 0;

figure(2);
plot(tval1*T, uval1(:,1),'LineWidth',mythick,'color',mycolor);
plot(tval2*T, uval2(:,1),'LineWidth',mythick,'color',mycolor);
plot(tval3*T, uval3(:,1)*2,'LineWidth',mythick,'color',mycolor);

figure(3);
plot(tval1*T, uval1(:,2)*3,'LineWidth',mythick,'color',mycolor);
plot(tval2*T, uval2(:,2)*3,'LineWidth',mythick,'color',mycolor);

