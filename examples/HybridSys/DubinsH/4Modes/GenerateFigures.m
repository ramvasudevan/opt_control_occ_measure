% Generate plots - Dubins car (4 modes), Minimum Time
clear;
close all;

scaling = 3;
x0 = [-0.8; 0.8; 0];
xT = [0.5; -0.2; 0];

figure(1); % traj
hold on;
figure(2); % control
subplot(1,2,1);
hold on;
subplot(1,2,2);
hold on;

%% Plot ground truth

% optimal trajectory
figure(1);
V = 1;
omega = 3;
r = V / omega;
p0 = [ x0(1); x0(2)-r ];
pT = [ xT(1); xT(2)+r ];
alpha = atan2( pT(1)-p0(1), pT(2)-p0(2) );
theta = alpha - acos( 2*r/ norm(pT-p0) );
xval1 = p0(1) + r * cos( pi/2:-0.01:theta );
yval1 = p0(2) + r * sin( pi/2:-0.01:theta );
xval2 = pT(1) - r * cos( theta:0.01:pi/2 );
yval2 = pT(2) - r * sin( theta:0.01:pi/2 );
t_total = 2*theta/omega + sqrt( norm(pT-p0)^2 - 4*r^2 ) / V;
plot( [xval1, xval2], [yval1, yval2], 'LineWidth', 4 );
plot(x0(1),x0(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot(xT(1),xT(2),'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
% auxiliary line
xval3 = r * cos( 0:0.01:2*pi );
yval3 = r * sin( 0:0.01:2*pi );
plot(xval3+p0(1), yval3+p0(2), 'k--');
plot(xval3+pT(1), yval3+pT(2), 'k--');

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
subplot(1,2,1);
plot([0,t_total],[1,1],'LineWidth',4);
xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$v(t)$','Interpreter','LaTex','FontSize',30);
xlim([0,2]);
ylim([0,1.1]);
set(gca,'XTick',[0,1,2]);
set(gca,'YTick',[0,1]);
set(gca, 'FontSize', 20);
box on;

subplot(1,2,2);
plot([0, theta/omega, theta/omega, t_total-theta/omega, t_total-theta/omega, t_total], [-1,-1,0,0,1,1],'LineWidth',4);
xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$\omega(t)$','Interpreter','LaTex','FontSize',30);
xlim([0,2]);
ylim([-1.2,1.2]);
set(gca,'XTick',[0,1,2]);
set(gca,'YTick',[-1,0,1]);
set(gca, 'FontSize', 20);
box on;

%% Plot our result
tc1 = plotdata('d12_1e5_angle',scaling,  3,  [.702,0,0]);
tc2 = plotdata('d8_1e5_angle',scaling,   2,  [.890,.290,.2]);
tc3 = plotdata('d6_1e5_angle',scaling,   1,  [.988,.553,.349]);

tcross = 1/3*( tc1 + tc2 + tc3 ) * scaling;

% Modes
figure(1);
plot([0,0],[-1,1],'k');
plot([-1,1],[0,0],'k');
text(-0.2,0.8,'I','FontSize',15);
text(0.5,0.8,'II','FontSize',15);
text(-0.5,-0.5,'III','FontSize',15);
text(0.5,-0.5,'IV','FontSize',15);

figure(2);
subplot(1,2,1);
plot([tcross(1),tcross(1)],[-1.5,1.5],'-k','LineWidth',1);
plot([tcross(2),tcross(2)],[-1.5,1.5],'-k','LineWidth',1);
text(0.56,0.5,'I','FontSize',15);
text(1.08,0.5,'II','FontSize',15);
text(1.55,0.5,'IV','FontSize',15);
subplot(1,2,2);
plot([tcross(1),tcross(1)],[-1.5,1.5],'-k','LineWidth',1);
plot([tcross(2),tcross(2)],[-1.5,1.5],'-k','LineWidth',1);
text(0.56,-0.5,'I','FontSize',15);
text(1.08,-0.5,'II','FontSize',15);
text(1.55,-0.5,'IV','FontSize',15);