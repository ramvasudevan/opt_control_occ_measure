function xdot = Hybrid_DIEq( t, x, u, J, var )

xval = x(1:2);
if (xval'*xval <= 0.3)
    uval = double(subs(u(1), var, [t;xval]));
else
    uval = double(subs(u(2), var, [t;xval]));
end

xdot = [ xval(2);
         uval;
         J(xval,uval) ];