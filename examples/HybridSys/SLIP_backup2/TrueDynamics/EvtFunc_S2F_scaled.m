function [value, isterminal, direction] = EvtFunc_S2F_scaled( ~, x )

value = x(1) - 1;
isterminal = 1;
direction = 1;