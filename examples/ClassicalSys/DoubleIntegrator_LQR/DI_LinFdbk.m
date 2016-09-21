function xdot = DI_LinFdbk( t, x, sol, Q, R )
    xval = x(1:2);
    idx = ceil( t * 100 + 1e-5 );
    u = - sol(idx).K * xval;
    xdot = [ x(2); u; xval' * Q * xval + R * u^2 ];
end