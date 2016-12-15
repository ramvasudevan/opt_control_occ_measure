function guard = rescale_guard_gpops(hS, y, a1, t1)
% g: x -> R is the guard in the original space
% x = a1*y1 + t1 is the linear transform from y1 to x
% rtn = hS(x) = hS(a1*y+t1)

a1 = repmat(a1', size(y,1), 1);
t1 = repmat(t1', size(y,1), 1);
x = a1 .* y + t1;

guard = ( hS(x') )';