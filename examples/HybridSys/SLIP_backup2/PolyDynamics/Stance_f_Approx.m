function [out] = Stance_f_Approx( x, ~ )
params = SLIPParams;
l0 = params.l0;
g = params.g;
k = params.k;
m = params.m;
umax = params.umax;

q1 = x(1);
q2 = x(2);
q3 = x(3);
q4 = x(4);

out = ...
    [ q2;
      0.1E2+0.5E0.*q3.^2+q1.*((-0.1E2)+q4.^2);
      q4;
      (1/6).*(6.*(3+(-3).*q1+q1.^2).*q3+(-1).*q3.^3+12.*((-2)+q1).*q2.*q4);
      (-1).*q2.*q3+(1/2).*((-2).*q1+q3.^2).*q4 ];