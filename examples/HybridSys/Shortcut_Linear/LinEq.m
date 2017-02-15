function xdot = LinEq( t, x, controller )

xval = x(1:2);
uval = controller(t,xval);
xdot = [ 0.5 * uval(1);
         uval(2) ];