function [fpoly] = Stance_Approx_f(x,params)

M = params.M;
k = params.k;
l0 = params.l0;
gg = params.gg;

polysin = @(x) x - x^3/6;
polycos = @(x) 1 - x^2 + x^4/24;
% polysin = @(x) x;
% polycos = @(x) 1 - x^2;

fpoly = [ x(2);
          -k/M*( x(1) - l0 ) - gg * polycos( x(3) ) + x(1) * x(4)^2;
          x(4);
          2 * x(2) * x(4) / l0  + gg * polysin( x(3) ) - l0;
          - x(2) * polysin( x(3) ) - x(1) * x(4) * polycos( x(3) ) ];