function xdot = DubinsEq( t, x, controller, J )

% polysin = @(x) x - x^3/6;
% polycos = @(x) 1 - x^2/2 + x^4/24;
polysin = @(x) x;
polycos = @(x) 1 - x^2/2;

xval = x(1:3);
% disp(xval);
uval = controller( t, xval );

if nargin < 4
    xdot = [ uval(1) * polycos(xval(3));
             uval(1) * polysin(xval(3));
             3 * uval(2) ];
else
    xdot = [ uval(1) * polycos(xval(3));
             uval(1) * polysin(xval(3));
             3 * uval(2);
             J(xval,uval) ];
end