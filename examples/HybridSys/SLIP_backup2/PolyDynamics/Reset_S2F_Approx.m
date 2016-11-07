function [ out ] = Reset_S2F_Approx(x,~)
params = SLIPParams;
l0 = params.l0;

% polysin = @(x) x;
% polycos = @(x) 1 - x^2/2;
polysin = @(x) -0.5 + sqrt(3)/2*(x+pi/6);
polycos = @(x) sqrt(3)/2 + 0.5*(x+pi/6);

out = [ x(5);
        -x(2) * polysin(x(3)) - l0 * x(4) * polycos(x(3));
        l0 * polycos(x(3));
        x(2) * polycos(x(3)) - l0 * x(4) * polysin(x(3)) ];