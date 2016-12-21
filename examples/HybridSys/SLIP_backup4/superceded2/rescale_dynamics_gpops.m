function dy = rescale_dynamics_gpops(f, y, a1, t1, a2)
% f: x -> R^n is the dynamics in the original space
% x = a1*y1 + t1 is the linear transform from y1 to x
% y2 = (x-t2)/a2 is the linear transform from x to y2
% outputs dy = 1 / a2 * f( a1 * x + t1)

a1 = repmat(a1', size(y,1), 1);
t1 = repmat(t1', size(y,1), 1);
x = a1 .* y + t1;
dx = ( f(x') )';

if nargin > 4
    a2 = repmat(a2', size(y,1), 1);
    dy = dx ./ a2;
else
    dy = dx;
end