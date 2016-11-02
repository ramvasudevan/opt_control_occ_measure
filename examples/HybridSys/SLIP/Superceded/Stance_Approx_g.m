function [gpoly] = Stance_Approx_g(~,params)

M = params.M;
k = params.k;
l0 = params.l0;
gg = params.gg;

polysin = @(x) x - x^3/6;
polycos = @(x) 1 - x^2 + x^4/24;
% polysin = @(x) x;
% polycos = @(x) 1 - x^2;

gpoly = [ 0;
          -k/M;
          0;
          0;
          0 ];