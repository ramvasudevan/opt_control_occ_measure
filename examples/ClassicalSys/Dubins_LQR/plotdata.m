function plotdata(filename,mythick,mycolor)

load(filename);

% trajectory
figure(3);
controller = @(tt,xx) [ double(subs(out.u{1,1},[t;xa],[tt;xx])); double(subs(out.u{1,2},[t;xa],[tt;xx])) ];
[ tval, xval ] = ode45( @(tt,xx) scaling*DubinsEq( tt, xx, controller ), [0,out.pval], x0{1} );
plot(xval(:,1),xval(:,2),'LineWidth',mythick,'color',mycolor);


uval = zeros( length(tval), 2 );
for i = 1 : length(tval)
    tt = tval(i);
    xx = xval(i,1:3)';
    uval(i,1) = double( subs(out.u{1,1}, [t;xa], [tt;xx]) );
    uval(i,2) = double( subs(out.u{1,2}, [t;xa], [tt;xx]) );
end
tval = scaling * tval;
% control 1
figure(1);
plot(tval,uval(:,1),'LineWidth',mythick,'color',mycolor);


% control 2
figure(2);
plot(tval,uval(:,2),'LineWidth',mythick,'color',mycolor);


