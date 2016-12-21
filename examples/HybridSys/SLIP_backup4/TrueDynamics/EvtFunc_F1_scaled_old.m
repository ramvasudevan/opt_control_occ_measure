function [value, isterminal, direction] = EvtFunc_F1_scaled_old( ~, x )
% Event function of Flight 1, scaled version

% value = x(3) - 1;
value = x(4) + 1;
isterminal = 1;
% direction = 1;
direction = -1;