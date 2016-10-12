% Generate plots - Double IntegratorH, minimum time
clear;
close all;

T = 5;
nSoln = 501;
r2 = 0.3;

figure(1); % traj
hold on;
figure(2); % control
hold on;

%% Plot ground truth

% modes
figure(1);
circx = -sqrt(r2):0.001:sqrt(r2);
circy = sqrt( r2 - circx .^ 2 );
h_area2 = area([-2;3],[-2,4;-2,4],'ShowBaseLine','off','FaceColor',[0.6 1 0.6]);
h_area = area(circx', [-circy; 2*circy]', 'BaseValue',0, 'ShowBaseLine','off','FaceColor',[1 0.6 0.6]);

% optimal trajectory
figure(1);

A = [ 0, 1; 0, 0 ];
B = [ 0; 1 ];
Q = eye(2);
R = 20;
F = 0 * eye(2);
tSpan = [ 0, T ];
tol = 1e-5;
x0 = [1;1];
sol = finiteLqr( tSpan,A,B,Q,R,F,nSoln,tol );
tspan = linspace( 0, T, nSoln );
[tval, xval] = ode45( @(t,x) DI_LinFdbk(t,x,sol,Q,R,T), tspan, [x0;0] );

plot(xval(:,1), xval(:,2), 'LineWidth', 4);

plot(x0(1),x0(2),'Marker','o','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
plot(0,0,'Marker','x','MarkerEdgeColor',[0 0.4470 0.7410],'MarkerSize',10,'LineWidth',4);
xlim([-1,2]);
ylim([-1,1]);
set(gca,'XTick',[-1,2]);
set(gca,'YTick',[-1,1]);
set(gca, 'FontSize', 20);
xlabel('$x_1$','Interpreter','LaTex','FontSize',30);
ylabel('$x_2$','Interpreter','LaTex','FontSize',30);
box on;

% input
figure(2);
uval = 0 * tval;
for i = 1 : length(tval)
    idx = ceil( tval(i) / T * (nSoln-1) + 1e-5 );
    uval(i) = - sol(idx).K * xval(i,1:2)';
end
plot(tval, uval, 'LineWidth', 4);

xlabel('$t$','Interpreter','LaTex','FontSize',30);
ylabel('$u(t)$','Interpreter','LaTex','FontSize',30);
xlim([0,1]*T);
ylim([-1,0.1]);
set(gca,'XTick',[0,0.2,0.4,0.6,0.8,1]*T);
set(gca,'YTick',[-1,0]);
set(gca, 'FontSize', 20);
box on;

disp(['Optimal cost = ', num2str(xval(end,end))]);

%% Plot our result
tc1 = plotdata('d12_1e4_T5',r2,T,  3,  [.702,0,0]);
tc2 = plotdata('d8_1e4_T5',r2,T,   2,  [.890,.290,.2]);
tc3 = plotdata('d6_1e4_T5',r2,T,   1,  [.988,.553,.349]);

% tc1 = plotdata('d12_1e4_T15',r2,T,  3,  [.702,0,0]);
% tc2 = plotdata('d8_1e4_T15',r2,T,   2,  [.890,.290,.2]);
% tc3 = plotdata('d6_1e4_T15',r2,T,   1,  [.988,.553,.349]);
% tcross = 1/3*( tc1 + tc2 + tc3 )*T;
% figure(2);
% plot( [tcross,tcross], [-1.5,1.5],'-k','LineWidth',1 );