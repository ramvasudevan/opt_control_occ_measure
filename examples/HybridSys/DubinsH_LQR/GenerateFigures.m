% Generate plots - Dubins car (4 modes), Minimum Time
clear;
close all;

scaling = 3;
x0 = [-0.8; 0.8; 0];
xT = [0.5; -0.2; 0];

figure(1); % traj
hold on;
figure(2); % control
% subplot(1,2,1);
hold on;
% subplot(1,2,2);
figure(3);
hold on;

%% Setup

% optimal trajectory
figure(1);
plot(x0(1),x0(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot(xT(1),xT(2),'Marker','.','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);

xlim([-1,1]);
ylim([-1,1]);
set(gca,'XTick',[-1,1]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
box on;

% Optimal control
figure(2);
% subplot(1,2,1);
% plot([0,t_total],[1,1],'LineWidth',4);
xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$V(t)$','Interpreter','LaTex','FontSize',30);
xlim([0,3]);
ylim([0,1.1]);
set(gca,'XTick',[0,1,2,3]);
set(gca,'YTick',[0,1]);
set(gca, 'FontSize', 20);
box on;

% subplot(1,2,2);
figure(3);
% plot([0, theta/omega, theta/omega, t_total-theta/omega, t_total-theta/omega, t_total], [-1,-1,0,0,1,1],'LineWidth',4);
xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$\omega(t)$','Interpreter','LaTex','FontSize',30);
xlim([0,3]);
ylim([-3.2,3.2]);
set(gca,'XTick',[0,1,2,3]);
set(gca,'YTick',[-3,0,3]);
set(gca, 'FontSize', 20);
box on;

%% Plot groundtruth result (gpops)
load('gpops_data');
figure(1);
plot(state(:,1), state(:,2),'LineWidth', 8, 'color', 'b');

figure(2);
% subplot(1,2,1);
plot(time, control(:,1), 'LineWidth', 8, 'color', 'b');
% subplot(1,2,2);
figure(3);
plot(time, control(:,2), 'LineWidth', 8, 'color', 'b');


%% Plot our result
[tc1,state] = plotdata('data_old/d10_1e5_v2w2',scaling,  6,  [.702,0,0]);
tc2 = plotdata('data_old/d8_1e5_v2w2',scaling,   4,  [.890,.290,.2]);
tc3 = plotdata('data_old/d6_1e5_v2w2',scaling,   2,  [.988,.553,.349]);

% Draw the car
figure(1);
idx = 1:10:101;
for i = 1 : length(idx)
    DrawCar(state(idx(i),:));
end

tcross = 1/3*( tc1 + tc2 + tc3 ) * scaling;

% Modes
figure(1);
plot([0,0],[-1,1],'k');
plot([-1,1],[0,0],'k');
text(-0.5,0.5,'I','FontSize',15);
text(0.5,0.5,'II','FontSize',15);
text(-0.5,-0.5,'III','FontSize',15);
text(0.5,-0.5,'IV','FontSize',15);
axis equal
xlim([-1,1]);
ylim([-1,1]);

figure(2);
% subplot(1,2,1);
plot([tcross(1),tcross(1)],[-1.5,1.5],'-k','LineWidth',1);
plot([tcross(2),tcross(2)],[-1.5,1.5],'-k','LineWidth',1);
text(0.56,0.8,'I','FontSize',15);
text(1.4,0.8,'II','FontSize',15);
text(2.4,0.8,'IV','FontSize',15);
% subplot(1,2,2);
figure(3);
plot([tcross(1),tcross(1)],[-4,4],'-k','LineWidth',1);
plot([tcross(2),tcross(2)],[-4,4],'-k','LineWidth',1);
text(0.56,1.5,'I','FontSize',15);
text(1.4,1.5,'II','FontSize',15);
text(2.4,1.5,'IV','FontSize',15);