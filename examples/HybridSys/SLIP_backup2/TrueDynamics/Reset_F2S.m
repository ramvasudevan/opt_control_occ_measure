function [ R ] = Reset_F2S(x,~)

params = SLIPParams;
alpha = params.alpha;
l0 = params.l0;

polysin = @(x) sin(x);
polycos = @(x) cos(x);


R = [ l0;
      -x(2) * polysin(alpha) + x(4) * polycos(alpha);
      alpha;
      -x(2) * polycos(alpha) / l0 - x(4) * polysin(alpha) / l0
      x(1) ];