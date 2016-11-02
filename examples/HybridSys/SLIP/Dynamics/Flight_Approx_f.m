function [fpoly] = Flight_Approx_f(x,params)

M = params.M;
k = params.k;
l0 = params.l0;
gg = params.gg;

fpoly = [ x(2);
          0;
          x(4);
          -gg ];