function [value, isterminal, direction] = EvtFunc21( ~, x, params )
% {y >= yR} -> {y = yR}

polysin = @(ang) ang - ang.^3/6 + ang.^5/120;
polycos = @(ang) 1 - ang.^2/2 + ang.^4/24;

y = x(1) * polycos(x(3));
yR = params.yR;

isterminal = 1;
direction = -1;
value = y - yR;