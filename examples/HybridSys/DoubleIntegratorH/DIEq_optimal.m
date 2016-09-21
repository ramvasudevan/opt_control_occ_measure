function xdot = DIEq_optimal( t, x, xs0 )

a = xs0(1);
b = xs0(2);

if ( t <= b + sqrt( a + b^2/2 ) )
    uval = -1;
else
    uval = 1;
end
xdot = [ x(2);
         uval ];