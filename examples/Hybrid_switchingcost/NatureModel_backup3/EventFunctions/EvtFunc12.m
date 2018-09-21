function [value, isterminal, direction] = EvtFunc12( ~, x, params )
% {y <= yR} -> {y >= yR}

polysin = @(ang) ang - ang^3/6 + ang^5/120;
polycos = @(ang) 1 - ang^2/2 + ang^4/24;

y = x(1) * polycos(x(3));
yR_hi = params.yR_hi-0.005;

isterminal = 1;
direction = 1;
value = y - yR_hi;