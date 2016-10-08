function xdot = Dubins_4MEq( t, x, u, J, var )

polysin = @(x) x;
polycos = @(x) 1 - x^2/2;
xval = x(1:3);

if ( xval(1) <= 0 ) && ( xval(2) >= 0 )
    uval1 = double( subs(u{1,1}, var, [t;xval]) );
    uval2 = double( subs(u{1,2}, var, [t;xval]) );
elseif ( xval(1) >= 0 ) && ( xval(2) >= 0 )
    uval1 = double( subs(u{2,1}, var, [t;xval]) );
    uval2 = double( subs(u{2,2}, var, [t;xval]) );
elseif ( xval(1) <= 0 ) && ( xval(2) <= 0 )
    uval1 = double( subs(u{3,1}, var, [t;xval]) );
    uval2 = double( subs(u{3,2}, var, [t;xval]) );
else
    uval1 = double( subs(u{4,1}, var, [t;xval]) );
    uval2 = double( subs(u{4,2}, var, [t;xval]) );
end

xdot = [ uval1 * polycos(1.5*xval(3));
         uval1 * polysin(1.5*xval(3));
         2 * uval2;
         J(xval) ];