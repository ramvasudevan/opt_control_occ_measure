function xdot = DIEq( t, x, controller )

uval = controller( t, x );
uval(uval<-1) = -1;
uval(uval>1) = 1;
xdot = [ x(2);
         uval ];
end

% function [value,isterminal,direction] = EventFcn(~,x)
% 
% 
% end