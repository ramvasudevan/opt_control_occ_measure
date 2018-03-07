function xdot = DIEq( t, x, u, J, var )

xval = x(1:2);
uval = double(subs(u(1), var, [t;xval]));

uval(uval>1) = 1;
uval(uval<-1) = -1;

xdot = [ xval(2);
         uval;
         J(xval,uval) ];