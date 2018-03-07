function plotdata(filename,mythick,mycolor)

load(filename);
T = 3;
figure(1);
controller = @(tt,xx) double(subs(out.u{1},[t;x],[tt;xx]));
ode_options = odeset('Events',@EventFcn);
[ tval, xval ] = ode45( @(tt,xx) T*DIEq( tt, xx, controller ), [0:0.01:T], x0, ode_options );
plot(xval(:,1), xval(:,2),'LineWidth',mythick,'color',mycolor);

figure(2);
controller = @(tt,xx) double(subs(out.u{1},[t;x],[tt;xx]));
uval = zeros( size(tval) );
for i = 1 : length(tval)
    uval(i) = controller( tval(i), xval(i,:)' );
end
uval(uval<-1) = -1;
uval(uval>1) = 1;
plot(tval*T, uval, 'LineWidth', mythick, 'color', mycolor);