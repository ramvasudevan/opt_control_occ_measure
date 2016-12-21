function [ R ] = Reset_S2F1(x,~)

R = [ x(5);
      -x(2) * sin(x(3)) - x(1) * x(4) * cos(x(3));
      x(1) * cos(x(3));
      x(2) * cos(x(3)) - x(1) * x(4) * sin(x(3)) ];