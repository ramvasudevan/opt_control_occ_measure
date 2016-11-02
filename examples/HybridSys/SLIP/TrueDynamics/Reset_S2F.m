function [ R ] = Reset_S2F(x,~)

% polysin = @(x) x - x^3/6;
% polycos = @(x) 1 - x^2 + x^4/24;
% polysin = @(x) x;
% polycos = @(x) 1 - x^2;
% polysin = @(x) -0.5 + 0.866 * (x + pi/6);
% polycos = @(x) 0.866 + 0.5 * (x + pi/6);
params = SLIPParams;
l0 = params.l0;

polysin = @(x) sin(x);
polycos = @(x) cos(x);

R = [ x(5);
      -x(2) * polysin(x(3)) - l0 * x(4) * polycos(x(3));
      l0 * polycos(x(3));
      x(2) * polycos(x(3)) - l0 * x(4) * polysin(x(3)) ];
