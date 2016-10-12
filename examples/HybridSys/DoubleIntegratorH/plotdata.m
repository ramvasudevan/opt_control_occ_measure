function tcross = plotdata(filename,r2,scaling,mythick,mycolor)

load(filename);

% trajectory
figure(1);
controller = [ out.u{1}; out.u{2} ];
ode_options = odeset('Events',@EventFcn);
[ tval, xval, time_event, ~, id_event ] = ode45(@(tt,xx) scaling * Hybrid_DIEq( tt, xx, controller, @(x) 1, [t;x{1}] ), ...
                       [0:0.001:out.pval], [xs0; 0], ode_options );
plot(xval(:,1), xval(:,2),'LineWidth',mythick,'color',mycolor);

idx = find( id_event == 2 );
tcross = [];
if ~isempty(idx)
    tcross = time_event( idx(1) );
end

% input
figure(2);
uval = zeros( size(tval) );
for i = 1 : length(tval)
    if (xval(i,1:2)'*xval(i,1:2) <= r2)
        uval(i) = double(subs(controller(1), [t;x{1}], [tval(i);xval(i,1:2)']));
    else
        uval(i) = double(subs(controller(2), [t;x{1}], [tval(i);xval(i,1:2)']));
    end
end
plot(tval*scaling, uval,'LineWidth',mythick,'color',mycolor);