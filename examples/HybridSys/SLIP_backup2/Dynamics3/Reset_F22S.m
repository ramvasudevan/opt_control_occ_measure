function [ R ] = Reset_F22S(x,params)

alpha = params.alpha;
l0 = params.l0;

R = [ l0;
      -x(2) * sin(alpha) + x(4) * cos(alpha);
      alpha;
      -x(2) * cos(alpha) / l0 - x(4) * sin(alpha) / l0
      x(1) ];