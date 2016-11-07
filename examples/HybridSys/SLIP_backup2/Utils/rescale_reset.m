function R = rescale_reset(r, y, a1, t1, a2, t2)
% r: x -> x is the reset map in the original space
% x = a1*y1 + t1 is the linear transform from y1 to x
% y2 = (x-t2)/a2 is the linear transform from x to y2
% outputs rtn = 1 / a2 * ( R( a1 * x + t1) - t2)

if size(y,1) == 1
    y = y';
end
x = a1 .* y + t1;
rx = r(x);
R = (rx - t2) .* (1 ./ a2);

end