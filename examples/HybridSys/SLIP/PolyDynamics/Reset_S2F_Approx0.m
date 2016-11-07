function [ out ] = Reset_S2F_Approx0(x,~)
% Same as Reset_S2F_Approx.m, but now sin and cos are taylor expanded
% around 0 (as oppose to -pi/6 in Reset_S2F_Approx.m)
params = SLIPParams;
l0 = params.l0;

polysin = @(xx) xx;
polycos = @(xx) 1 - xx^2/2;
% polysin = @(x) -0.5 + sqrt(3)/2*(x+pi/6);
% polycos = @(x) sqrt(3)/2 + 0.5*(x+pi/6);

out = [ x(5);
        -x(2) * polysin(x(3)) - l0 * x(4) * polycos(x(3));
        l0 * polycos(x(3));
        x(2) * polycos(x(3)) - l0 * x(4) * polysin(x(3)) ];