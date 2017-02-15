% Generate sample trajectory

clear;
close all;
l0 = 0.2;           % maximum leg length
lk = 0.11;          % spring length
la = l0 - lk;       % actuator length
w = 0.02;
h = 0.035;

theta = pi/6;
springcoord( [ 0, 0 ] , [ 0, lk ], 3, lk*2, 0.02);

figure;
hold on;
%% Plot left
Opos = [ 0.2, 0 ];

Mpos = Opos + l0 * [ -sin(theta), cos(theta) ];
Kpos = Opos + lk * [ -sin(theta), cos(theta) ];

R = [ cos(theta), -sin(theta), Mpos(1);
      sin(theta),  cos(theta), Mpos(2);
               0,           0,       1 ];

% Mass
th = 0:pi/50:2*pi;
xvec = Mpos(1) + 0.02*cos(th);
yvec = Mpos(2) + 0.02*sin(th);
patch(xvec, yvec, 1, 'FaceColor', [0.8,0.8,0.8], 'LineWidth',2);

% Spring
[xvec, yvec] = springcoord( Kpos, Opos );
plot(xvec, yvec, 'k' ,'LineWidth',2);

% Actuator
line1 = [ 0, 0;
          0, -la;
          1, 1 ];
tmp = R*line1;
plot(tmp(1,:),tmp(2,:),'k','LineWidth',2);

axis equal

% auxiliary lines
plot([0,1],[0,0],'k','Linewidth',2);
line1 = [ -0.05, 0;
          0,     0;
          1,     1 ];
line2 = [ -0.05, 0;
          -0.2,     -0.2;
          1,     1 ];
line3 = [ 0,     0.04;
          -la/2, -la/2;
          1,     1 ];
line4 = [ 0,            0.04;
          -la/2-w/3-h,  -la/2-w/3-h;
          1,            1 ];
tmp = R*line1;
plot(tmp(1,:),tmp(2,:),'--k','LineWidth', 1);
tmp = R*line2;
plot(tmp(1,:),tmp(2,:),'--k','LineWidth', 1);
% tmp = R*line3;
% plot(tmp(1,:),tmp(2,:),'--k','LineWidth', 1);
% tmp = R*line4;
% plot(tmp(1,:),tmp(2,:),'--k','LineWidth', 1);
plot([0.2, 0.2],[0, 0.12], '--k','LineWidth', 1);
%% Plot right

Opos = [ 0.5, 0 ];

Mpos = Opos + l0 * [ -sin(theta), cos(theta) ];
Kpos = Opos + lk * [ -sin(theta), cos(theta) ];

R = [ cos(theta), -sin(theta), Mpos(1);
      sin(theta),  cos(theta), Mpos(2);
               0,           0,       1 ];

% Mass
th = 0:pi/50:2*pi;
xvec = Mpos(1) + 0.02*cos(th);
yvec = Mpos(2) + 0.02*sin(th);
patch(xvec, yvec, 1, 'FaceColor', [0.8,0.8,0.8], 'LineWidth',2);

% Spring
[xvec, yvec] = springcoord( Kpos, Opos );
plot(xvec, yvec, 'k' ,'LineWidth',2);

% Actuator
line1 = [ 0, 0;
          0, -la/2;
          1, 1 ];
line2 = [ -w/2,     -w/2,   w/2,    w/2;
          -la/2-h,  -la/2,  -la/2,  -la/2-h;
          1         1,      1,      1 ];
line3 = [ -w/6,      w/6,        w/6,           -w/6,           -w/6;
          -la/2-w/3, -la/2-w/3,  -la/2-w/3-h,   -la/2-w/3-h,    -la/2-w/3;
          1,         1,          1,             1,              1 ];
line4 = [ 0,            0;
          -la/2-w/3-h,  -la;
          1,            1 ];
tmp = R*line1;
plot(tmp(1,:),tmp(2,:),'k','LineWidth',2);
tmp = R*line2;
plot(tmp(1,:),tmp(2,:),'k','LineWidth',2);
tmp = R*line3;
plot(tmp(1,:),tmp(2,:),'k','LineWidth',2);
tmp = R*line4;
plot(tmp(1,:),tmp(2,:),'k','LineWidth',2);

% auxiliary lines
plot([0,1],[0,0],'k','Linewidth',2);
line1 = [ -0.05, 0;
          0,     0;
          1,     1 ];
line2 = [ -0.05, 0;
          -0.2,     -0.2;
          1,     1 ];
line3 = [ 0,     0.04;
          -la/2, -la/2;
          1,     1 ];
line4 = [ 0,            0.04;
          -la/2-w/3-h,  -la/2-w/3-h;
          1,            1 ];
tmp = R*line1;
plot(tmp(1,:),tmp(2,:),'--k','LineWidth', 1);
tmp = R*line2;
plot(tmp(1,:),tmp(2,:),'--k','LineWidth', 1);
tmp = R*line3;
plot(tmp(1,:),tmp(2,:),'--k','LineWidth', 1);
tmp = R*line4;
plot(tmp(1,:),tmp(2,:),'--k','LineWidth', 1);
plot([0.5, 0.5],[0, 0.12], '--k','LineWidth', 1);
%%


axis equal
% xlim([0,0.6]);
xlim([0,0.3]);
% xlim([0.3,0.6]);
ylim([-0.05,0.25]);
axis off
