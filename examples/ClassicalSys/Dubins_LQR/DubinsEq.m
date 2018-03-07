function xdot = DubinsEq( t, x, u, var )

polysin = @(x) x;
polycos = @(x) 1 - x^2/2;
xval = x(1:3);
uval1 = double( subs(u{1,1}, var, [t;xval]) );
uval2 = double( subs(u{1,2}, var, [t;xval]) );

uval1( uval1 > 1 ) = 1;
uval1( uval1 < -1 ) = -1;
uval2( uval2 > 1 ) = 1;
uval2( uval2 < -1 ) = -1;

xdot = [ uval1 * polycos(xval(3));
         uval1 * polysin(xval(3));
         3 * uval2 ];