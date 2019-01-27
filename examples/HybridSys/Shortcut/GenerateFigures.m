% Generate plots - Shortcut (3 modes), minimum time
clear;
close all;

x0 = [-0.8; 0.8; 0];
xT = [0.8; -0.8; 0];

figure(1); % traj
hold on;
figure(2); % control
% subplot(1,2,1);
hold on;
% subplot(1,2,2);
figure(3);
hold on;


%% Setup
figure(1);

plot3(x0(1),x0(2),0,'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot3(xT(1),xT(2),0,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot3([1,1,-1,-1,1],[1,-1,-1,1,1],[0,0,0,0,0],'k');
plot3([-1,1],[0,0],[0,0],'k');
plot3([1,1],[-1,1],[1,1],'k');

xlim([-1,1]);
ylim([-1,1.05]);
zlim([0,1]);
set(gca,'XTick',[-1,1]);
set(gca,'YTick',[-1,0,1]);
set(gca, 'FontSize', 20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
bg_color = get(gca,'Color');
set(gca,'ZColor',bg_color,'ZTick',[]);
text(1.03,-1.1,1,'-1','FontSize',20);
text(1.05,1,1.05,'1','FontSize',20);
view([-0.7,-1,1.5]);

figure(2);
xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$v(t)$','Interpreter','LaTex','FontSize',30);
% xlim([0,2]);
xlim([0,1.6]);
% ylim([-0.1,2.1]);
ylim([0.5,2.1]);
set(gca,'XTick',[0,1,2,3]);
set(gca,'YTick',[0,1,2]);
set(gca, 'FontSize', 20);
box on;

figure(3);
xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$\omega(t)$','Interpreter','LaTex','FontSize',30);
xlim([0,2]);
ylim([-3.2,3.2]);
set(gca,'XTick',[0,1,2,3]);
set(gca,'YTick',[-3,0,3]);
set(gca, 'FontSize', 20);
box on;

%% Plot gpops result, 1-3-2
% load('gpops_data');
% solution = output.result.solution;
% 
% time1 = solution.phase(1).time;
% state1 = solution.phase(1).state;
% control1 = solution.phase(1).control;
% 
% time2 = solution.phase(2).time;
% state2 = solution.phase(2).state;
% control2 = solution.phase(2).control;
% 
% time3 = solution.phase(3).time;
% state3 = solution.phase(3).state;
% control3 = solution.phase(3).control;
% 
% 
% figure(1);
% plot3(state1(:,1), state1(:,2), state1(:,1)*0, 'LineWidth', 4, 'color', 'b');
% plot3(state3(:,1), state3(:,2), state3(:,1)*0, 'LineWidth', 4, 'color', 'b');
% plot3(state2*0+1, state2,state2*0+1, 'LineWidth', 4, 'color', 'b');
% 
% plot3([state1(end,1), 1], [state1(end,2),state2(1)], [0,1], 'LineWidth', 2, 'color', [0.5,0.5,0.5]);
% q = [ (state1(end,1)+1)/2, (state1(end,2)+state2(1))/2, 1/2];
% dq = [ 1-state1(end,1), state2(1)-state1(end,2), 1 ];
% quiver3(q(1), q(2), q(3), dq(1)/5, dq(2)/5, dq(3)/5, 'MaxHeadSize', 20, 'LineWidth', 2, 'color', [0.5,0.5,0.5]);
% plot3([1, 0.6],[-1,-0.8],[1,0], 'LineWidth', 2, 'color', [0.5,0.5,0.5]);
% q = [ 2/3+0.2, -2/3-0.8/3, 2/3];
% dq = [ -0.4, 0.2, -1 ];
% quiver3(q(1), q(2), q(3), dq(1)/3, dq(2)/3, dq(3)/3, 'MaxHeadSize', 20, 'LineWidth', 2, 'color', [0.5,0.5,0.5]);
% 
% figure(2);
% plot(time1, control1(:,1), 'LineWidth', 4, 'color', 'b');
% plot(time2, control2(:,1), 'LineWidth', 4, 'color', 'b');
% plot(time3, control3(:,1), 'LineWidth', 4, 'color', 'b');
% figure(3);
% plot(time1, control1(:,2), 'LineWidth', 4, 'color', 'b');
% plot(time3, control3(:,2), 'LineWidth', 4, 'color', 'b');

%% Plot gpops result, 1-2
% load('gpops_data2');
% solution = output.result.solution;
% 
% time1 = solution.phase(1).time;
% state1 = solution.phase(1).state;
% control1 = solution.phase(1).control;
% 
% time2 = solution.phase(2).time;
% state2 = solution.phase(2).state;
% control2 = solution.phase(2).control;
% 
% figure(1);
% plot3(state1(:,1), state1(:,2), state1(:,1)*0, '--', 'LineWidth', 1, 'color', 'b');
% plot3(state2(:,1), state2(:,2), state2(:,1)*0, '--', 'LineWidth', 1, 'color', 'b');
% 
% figure(2);
% plot(time1, control1(:,1), '--', 'LineWidth', 1, 'color', 'b');
% plot(time2, control2(:,1), '--', 'LineWidth', 1, 'color', 'b');
% figure(3);
% plot(time1, control1(:,2), '--', 'LineWidth', 1, 'color', 'b');
% plot(time2, control2(:,2), '--', 'LineWidth', 1, 'color', 'b');

%% Ground truth result
mythick = 4;
mycolor = [0,0,1];
J = @(xx) 1;

T = 3;
figure(1);
% In mode 1
controller1 = @(tt,xx) [1;1];
ode_options = odeset('Events', @EventFcn_1);
[ tval1, xval1 ] = ode45( @(tt,xx) T*DubinsEq( tt, xx, controller1 ), [0:0.001:1], x0, ode_options );
h_traj1 = plot3( xval1(:,1), xval1(:,2), xval1(:,1)*0,'LineWidth',mythick,'color',mycolor );

% In mode 3
ode_options = odeset('Events', @EventFcn_3);
[ tval3, xval3 ] = ode45( @(tt,xx) -2*T, ...
                          [tval1(end):0.001:1], 1, ode_options );
h_traj3 = plot3( xval3*0+1, xval3, xval3*0+1,'LineWidth',mythick,'color',mycolor );

% In mode 2
controller2 = @(tt,xx) [ 1; 0 ];
ode_options = odeset('Events', @EventFcn_2);
[ tval2, xval2 ] = ode45( @(tt,xx) T*DubinsEq( tt, xx, controller2 ), ...
                          [tval3(end):0.001:1], [ 0.6; -0.8; 0 ], ode_options );
h_traj2 = plot3( xval2(:,1), xval2(:,2), xval2(:,1)*0,'LineWidth',mythick,'color',mycolor );

% Control
figure(2);
plot([tval1(1), tval1(end)]*T, [1, 1],'LineWidth',mythick,'color',mycolor);
plot([tval2(1), tval2(end)]*T, [1, 1],'LineWidth',mythick,'color',mycolor);
plot([tval3(1), tval3(end)]*T, [2, 2],'LineWidth',mythick,'color',mycolor);

figure(3);
plot([tval1(1), tval1(end)]*T, [3, 3],'LineWidth',mythick,'color',mycolor);
plot([tval2(1), tval2(end)]*T, [0, 0],'LineWidth',mythick,'color',mycolor);

% Arrows
figure(1);
plot3([xval1(end,1), 1], [xval1(end,2),xval3(1)], [0,1], 'LineWidth', 2, 'color', [0.5,0.5,0.5]);
q = [ (xval1(end,1)+1)/2, (xval1(end,2)+xval3(1))/2, 1/2];
dq = [ 1-xval1(end,1), xval3(1)-xval1(end,2), 1 ];
quiver3(q(1), q(2), q(3), dq(1)/5, dq(2)/5, dq(3)/5, 'MaxHeadSize', 20, 'LineWidth', 2, 'color', [0.5,0.5,0.5]);
plot3([1, 0.6],[-1,-0.8],[1,0], 'LineWidth', 2, 'color', [0.5,0.5,0.5]);
q = [ 2/3+0.2, -2/3-0.8/3, 2/3];
dq = [ -0.4, 0.2, -1 ];
quiver3(q(1), q(2), q(3), dq(1)/3, dq(2)/3, dq(3)/3, 'MaxHeadSize', 20, 'LineWidth', 2, 'color', [0.5,0.5,0.5]);

%% Plot our result
plotdata('data/d10_1e4',3, [.702,0,0]);
plotdata('data/d8_1e4', 2, [.890,.290,.2])
plotdata('data/d6_1e4', 1, [.988,.553,.349]);


%% Modes
figure(1);
text(-.1,0.5,0,'Mode 1','FontSize',15);
text(-.1,-0.5,0,'Mode 2','FontSize',15);
text(1,0,1.1,'Mode 3','FontSize',15);

figure(2);
plot([tval1(end), tval1(end)]*T, [-1, 3], '-k', 'LineWidth', 1);
plot([tval3(end), tval3(end)]*T, [-1, 3], '-k', 'LineWidth', 1);

figure(3);
plot([tval1(end), tval1(end)]*T, [-4, 4], '-k', 'LineWidth', 1);
plot([tval3(end), tval3(end)]*T, [-4, 4], '-k', 'LineWidth', 1);