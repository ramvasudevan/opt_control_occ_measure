% Generate plots - Double IntegratorH, minimum time
clear;
close all;

x0 = [ 0.3; 1 ];
r2 = 0.3;
scaling = 5;
t1 = x0(2) + sqrt(x0(1) + x0(2)^2/2);
t2 = sqrt(x0(1) + x0(2)^2/2);

figure(1); % traj
hold on;
figure(2); % control
hold on;

%% Plot ground truth

% modes
figure(1);
circx = -sqrt(r2):0.001:sqrt(r2);
circy = sqrt( r2 - circx .^ 2 );
% h_area2 = area([-2;3],[-2,4;-2,4],'ShowBaseLine','off','FaceColor',[0.6 1 0.6]);
% h_area = area(circx', [-circy; 2*circy]', 'BaseValue',0, 'ShowBaseLine','off','FaceColor',[1 0.6 0.6]);

h_area2 = area([-2;3],[-2,4;-2,4],'ShowBaseLine','off','FaceColor',[1 1 1]);
h_area = area(circx', [-circy; 2*circy]', 'BaseValue',0, 'ShowBaseLine','off','FaceColor',[1 1 1]);


% optimal trajectory
figure(1);
controller = @(tt,xx) 1-2*(tt <= t1);
% ode_options = odeset('Events',@EventFcn);
[ tval1, xvalA1 ] = ode45( @(tt,xx) DIEq( tt, xx, controller ), ...
                       [0:0.001:t1], x0 );
[ tval2, xvalA2 ] = ode45( @(tt,xx) DIEq( tt, xx, controller ), ...
                                                [t1:0.001:(t1+t2)],...
                                                xvalA1(end,:) );
xvalA1 = [xvalA1; xvalA2];
h_traj2 = plot(xvalA1(:,1), xvalA1(:,2), 'LineWidth', 4);
plot(x0(1),x0(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot(0,0,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
xlim([-1,1.5]);
ylim([-1,1]);
set(gca,'XTick',[-1,1.5]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
box on;

% input
figure(2);
plot([0,t1,t1,t1+t2],[-1,-1,1,1],'LineWidth',4);

xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$u(t)$','Interpreter','LaTex','FontSize',30);
set(gca,'XTick',[0,1,2,3]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
box on;

%% Plot our result
tc1 = plotdata('d12_1e4',r2,scaling,  3,  [.702,0,0]);
% tc2 = plotdata('d8_1e4',r2,scaling,   2,  [.890,.290,.2]);
% tc3 = plotdata('d6_1e4',r2,scaling,   1,  [.988,.553,.349]);

tcross = 1/3*( tc1 + tc2 + tc3 )*scaling-0.03;
figure(2);
plot( [tcross,tcross], [-1.5,1.5],'-k','LineWidth',1 );