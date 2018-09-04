% Plot Van der Pol limit cycle
close all;
figure(1);
hold on;
% axis
line([0,0],[-1,1],'Color','k');
line([-1,-0.05],[0,0],'Color',[35,139,69]/255, 'linewidth',3);

% Limit cycle
% [t,y] = ode45( @(t,xx) VDP_Dyn(xx), [0:0.01:1000], [0.8,0]  );
% plot(y(end-1000:end,1), y(end-1000:end,2), 'linewidth', 3);

% vector field
[ xval, yval ] = meshgrid( -1:0.1:1, -1:0.1:1 );
uval = yval * 3 / 2.5;
vval = (0.3*(1-(xval*2.5).^2).*yval * 3 - xval*2.5) / 3;
uu = uval ./ sqrt( uval.^2 + vval.^2 );
vv = vval ./ sqrt( uval.^2 + vval.^2 );
quiver(xval, yval, uu, vv, 'AutoScaleFactor', 0.3);

% [ xval, yval ] = meshgrid( -1:0.1:1, -1:0.1:1 );
% uval = yval;
% vval = (0.3*(1-(xval).^2).*yval - xval) / 3;
% uu = uval ./ sqrt( uval.^2 + vval.^2 );
% vv = vval ./ sqrt( uval.^2 + vval.^2 );
% quiver(xval, yval, uu, vv, 'AutoScaleFactor', 0.3);


cm = ...
    [ 255, 255, 204
      255, 237, 160
      254, 217, 118
      254, 178,  76
      253, 141,  60
      252,  78,  42
      227,  26,  28
      189,   0,  38
      120,   0,  38
      120,   0,  38
      120,   0,  38 ] / 255;

axis equal
xlim([-1,1]);
ylim([-1,1]);
box on;

cnt = 1;
for offset = -0.8 : 0.1 : -0.1
    set(gcf,'DefaultAxesColorOrder', cm / 255);
    [basis, moments] = run_VDP_ICopt( offset );
    x0 = double( [ moments(2), 0 ] );
    [tt,yy] = ode45( @(t,xx) VDP_Dyn(xx), [0:0.01:6], x0 );
    plot(yy(:,1), yy(:,2),'linewidth',3, 'Color', cm(end-cnt,:));
    cnt = cnt + 1;
%     pause;
end

crossy = [ -0.6991, -0.6009, -0.5, -0.3994, -0.3, -0.1976, -0.09626 ];

ptcolor = [ 66, 146, 198 ] / 255;
plot(zeros(1,7), crossy, '.', 'Color', 'k', 'MarkerSize',25);

% % test
% x0 = [0, -0.4];
% [tt,yy] = ode45( @(tt,xx) -VDP_Dyn(xx), [0:0.01:6], x0 );
% plot(yy(:,1), yy(:,2), 'linewidth', 3);