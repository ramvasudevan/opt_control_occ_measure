function [value, isterminal, direction] = EvtFunc_F2( ~, x )

params = SLIPParams;
value = x(3) - params.yR;
isterminal = 1;
direction = -1;