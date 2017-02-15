
clear;
close all;

load('d6');
scaling = 3;

figure(1); % traj
hold on;
% figure(2); % control
% subplot(1,2,1);
% hold on;
% subplot(1,2,2);
% hold on;

%% Plot

figure(1);
plot(-0.8,0.8,'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot(0.8,-0.8,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
xlim([-1,1]);
ylim([-1,1]);
set(gca,'XTick',[-1,1]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
box on;

plot([-1,1],[0,0],'k');
text(0,0.5,'I','FontSize',15);
text(0,-0.5,'II','FontSize',15);

% Plot trajectories
controller0 = @(t,x) [1;1];
controller1 = @(tt,xx) [ double(subs(out.u{1,1},[t;x{1}],[tt;xx]))/2 + 0.5; double(subs(out.u{1,2},[t;x{1}],[tt;xx])) ];
controller2 = @(tt,xx) [ double(subs(out.u{2,1},[t;x{2}],[tt;xx]))/2 + 0.5; double(subs(out.u{2,2},[t;x{1}],[tt;xx])) ];
controller3 = @(tt,xx) [ double(subs(out.u{3,1},[t;x{3}],[tt;xx])) ];

ode_options = odeset('Events',@EventFcn);
[ tval, xval ] = ode45( @(tt,xx) scaling*DubinsEq( tt, xx, controller1 ), [0:0.01:1], x0{1}, ode_options );
figure(1);
h_traj = plot(xval(:,1), xval(:,2),'LineWidth',4,'color',[0 0.4470 0.7410]);

% ode_options3 = odeset('Events',@EventFcn3);
% [ tval3, xval3 ] = ode45( @(tt,xx) scaling*controller3(tt,xx), [tval(end):0.01:0.5], [-1], ode_options3 );
% figure(3);
% plot(tval3, xval3);

[ tval2, xval2 ] = ode45( @(tt,xx) scaling*DubinsEq( tt, xx, controller2 ), [0.1955:0.01:1], [0.6;-0.8;0], ode_options );
figure(1);
h_traj2 = plot(xval2(:,1), xval2(:,2),'LineWidth',4,'color',[0 0.4470 0.7410]);


