function xdot = DubinsEq( t, x, controller, J )

% polysin = @(x) x - x^3/6;
% polycos = @(x) 1 - x^2/2 + x^4/24;
% polysin = @(x) x;
% polycos = @(x) 1 - x^2/2;
polysin = @(x) sin(x);
polycos = @(x) cos(x);

xval = x(1:3);
uval = controller( t, xval );
if uval(1)>1
    uval(1) = 1;
end
if uval(2)>1
    uval(2) = 1;
end

if nargin < 4
    xdot = [ uval(1) * polycos(1.5*xval(3));
             uval(1) * polysin(1.5*xval(3));
             2 * uval(2) ];
else
    xdot = [ uval(1) * polycos(1.5*xval(3));
             uval(1) * polysin(1.5*xval(3));
             2 * uval(2);
             J(xval) ];
end