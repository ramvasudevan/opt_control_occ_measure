function [fpoly] = Stance_f(x,~)
% Degree 3

l0 = 1;
g = 1;
k = 10;
m = 1;

q1 = x(1);
q2 = x(2);
q3 = x(3);
q4 = x(4);

fpoly = ...
    [ q2;
      -k/m * (q1 - l0) - g * cos(q3) + q1 * q4^2  +  k/m*0.1;
      q4;
      -2 * q2 * q4 / q1 + g / q1 * sin(q3);
      -q2 * sin(q3) - q1 * q4 * cos(q3) ];
