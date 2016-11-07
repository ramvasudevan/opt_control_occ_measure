function [value, isterminal, direction] = EvtFunc_F2S_scaled( ~, x )

value = x(3) + 1;
isterminal = 1;
direction = -1;