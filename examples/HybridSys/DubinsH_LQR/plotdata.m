function [tcross, state] = plotdata(filename,scaling,mythick,mycolor)

load(filename);
disp(['Processing ',filename]);

% trajectory
figure(1);

J = @(xx,uu) (xx(1)-0.5)^2 + (xx(2)+0.2)^2 + uu(1)^2 + uu(2)^2;
% J = @(xx,uu) 1;
ode_options = odeset('Events',@EventFcn);
[tval, xval, time_event, ~, id_event] = ode45( @(tt,xx) scaling*Dubins_4MEq_old( tt, xx, out.u, J, [t;xa] ), ...
                     [0:0.01:1], [x0{1};0], ode_options);
plot(xval(:,1), xval(:,2),'LineWidth',mythick,'color',mycolor);

tcross = zeros(1,2);
tcross(1) = time_event( id_event == 1 );
tcross(2) = time_event( id_event == 2 );

% input
figure(2);
uval = zeros( length(tval), 2 );
for i = 1 : length(tval)
    tt = tval(i);
    xx = xval(i,1:3)';
    if ( xx(1) <= 0 ) && ( xx(2) >= 0 )
        uval(i,1) = double( subs(out.u{1,1}, [t;xa], [tt;xx]) );
        uval(i,2) = double( subs(out.u{1,2}, [t;xa], [tt;xx]) );
    elseif ( xx(1) >= 0 ) && ( xx(2) >= 0 )
        uval(i,1) = double( subs(out.u{2,1}, [t;xa], [tt;xx]) );
        uval(i,2) = double( subs(out.u{2,2}, [t;xa], [tt;xx]) );
    elseif ( xx(1) <= 0 ) && ( xx(2) <= 0 )
        uval(i,1) = double( subs(out.u{3,1}, [t;xa], [tt;xx]) );
        uval(i,2) = double( subs(out.u{3,2}, [t;xa], [tt;xx]) );
    else
        uval(i,1) = double( subs(out.u{4,1}, [t;xa], [tt;xx]) );
        uval(i,2) = double( subs(out.u{4,2}, [t;xa], [tt;xx]) );
    end
end

uval( uval > 1 ) = 1;
uval( uval < -1 ) = -1;
uval(:,2) = uval(:,2)*3;

% subplot(1,2,1);
figure(2);
plot(tval*scaling,uval(:,1),'LineWidth',mythick,'color',mycolor);
% subplot(1,2,2);
figure(3);
plot(tval*scaling,uval(:,2),'LineWidth',mythick,'color',mycolor);

state = [ xval(:,1), xval(:,2), 1.5*xval(:,3) ];