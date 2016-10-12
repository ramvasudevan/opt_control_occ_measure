% Test optimal trajectory

scaling = 3;
d = 6;
controller1 = @(t,x) [1;1];
controller2 = @(t,x) 1;
controller3 = @(t,x) [1,0];

x0 = [ -0.8; 0.8; 0; 0 ];
figure(2);
hold on;
ode_options = odeset('Events',@EventFcn);
[ tval1, xval1 ] = ode45( @(tt,xx) scaling*DubinsEq( tt, xx, controller1, (@(tmp) 1) ), [0:0.01:1], x0, ode_options );
plot(xval1(:,1), xval1(:,2),'LineWidth',4);