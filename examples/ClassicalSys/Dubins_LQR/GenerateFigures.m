% Generate plots - Dubins Car with LQR

figure(1); % control1
hold on;
ylim([0,1.2]);
xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$V(t)$','Interpreter','LaTex','FontSize',30);
set(gca, 'FontSize', 20);
box on;

figure(2); % control2
hold on;
ylim([-1.1,1.1]*3);
xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$\omega(t)$','Interpreter','LaTex','FontSize',30);
set(gca, 'FontSize', 20)
box on;

figure(3); % trajectory
hold on;
plot(-0.8,0.8,'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot(0.5,-0.4,'Marker','.','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
xlim([-1,1]);
ylim([-1,1]);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
set(gca,'XTick',[-1,1]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
box on;

plotdata('d10_1e5',3,[.702,0,0]);
plotdata('d8_1e5',2,[.890,.290,.2]);
plotdata('d6_1e5',1,[.988,.553,.349]);