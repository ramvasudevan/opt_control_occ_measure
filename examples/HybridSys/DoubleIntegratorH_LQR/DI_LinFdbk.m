function xdot = DI_LinFdbk( t, x, sol, Q, R, T )
    xval = x(1:2);
    nSoln = length(sol);
    idx = ceil( t / T * (nSoln-1) + 1e-5 );
    u = - sol(idx).K * xval;
    if (u>1) || (u<-1)
        error('Wrong!!');
    end
    xdot = [ x(2); u; xval' * Q * xval + R * u^2 ];
end