% Generate plots - Double Integrator, minimum time
clear;
close all;

x0 = [ 0.3; 1 ];
t1 = x0(2) + sqrt(x0(1) + x0(2)^2/2);
t2 = sqrt(x0(1) + x0(2)^2/2);

figure(1); % traj
hold on;
figure(2); % control
hold on;


%% Plot
figure(1);
controller = @(tt,xx) 1-2*(tt <= t1);
[ ~, xvalA1 ] = ode45( @(tt,xx) DIEq( tt, xx, controller ), ...
                       [0:0.001:t1], x0 );
[ ~, xvalA2 ] = ode45( @(tt,xx) DIEq( tt, xx, controller ), ...
                       [t1:0.001:(t1+t2)],...
                       xvalA1(end,:) );
xvalA1 = [xvalA1; xvalA2];
h_traj2 = plot(xvalA1(:,1), xvalA1(:,2), 'LineWidth', 4);

xlim([-1,1.5]);
ylim([-1,1]);
set(gca,'XTick',[-1,1.5]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
box on;


figure(2);
plot([0,t1,t1,t1+t2],[-1,-1,1,1],'LineWidth',4);
xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$u(t)$','Interpreter','LaTex','FontSize',30);
ylim([-1.2,1.2]);
set(gca,'XTick',[0,1,2,3]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
box on;

plotdata('DI_d12',3,[.702,0,0]);
plotdata('DI_d8',2,[.890,.290,.2]);
plotdata('DI_d6',1,[.988,.553,.349]);


figure(1);
legend('2k=12','2k=8','2k=6');
plot(x0(1),x0(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot(0,0,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
figure(2);
legend('2k=12','2k=8','2k=6');