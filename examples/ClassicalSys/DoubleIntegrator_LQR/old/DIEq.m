function xdot = DIEq( t, x, controller )
Q = eye(2);
R = 20;
xval = x(1:2);

uval = controller( t, xval );
xdot = [ xval(2);
         uval;
         xval' * Q * xval + uval*R*uval ];
end

% function [value,isterminal,direction] = EventFcn(~,x)
% 
% 
% end