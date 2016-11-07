function [value, isterminal, direction] = EvtFunc_F2S_scaled( ~, x )

value(1) = x(3) + 1;
value(2) = x(3) + 1;
isterminal(1) = 0;
isterminal(2) = 1;
direction(1) = 1;
direction(2) = -1;