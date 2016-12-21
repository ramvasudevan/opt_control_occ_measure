function [out] = Stance_f(x,~)
params = SLIPParams;
l0 = params.l0;
g = params.g;
k = params.k;
m = params.m;
umax = params.umax;

if size(x,1) == 1
    x = x';
end
q1 = x(1,:);
q2 = x(2,:);
q3 = x(3,:);
q4 = x(4,:);

out = ...
    [ q2;
      -k/m .* (q1 - l0) - g .* cos(q3) + q1 .* q4.^2  +  k/m * umax / 2;
      q4;
      -2 .* q2 .* q4 ./ q1 + g ./ q1 .* sin(q3);
      -q2 .* sin(q3) - q1 .* q4 .* cos(q3) ];
