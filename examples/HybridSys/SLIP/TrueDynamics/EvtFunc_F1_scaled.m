function [value, isterminal, direction] = EvtFunc_F1_scaled( ~, x )
% Event function of Flight 1, scaled version

value = x(4) + 1;
isterminal = 1;
direction = -1;