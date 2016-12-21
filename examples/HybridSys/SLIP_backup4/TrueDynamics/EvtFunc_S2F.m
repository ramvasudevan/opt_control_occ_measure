function [value, isterminal, direction] = EvtFunc_S2F( ~, x )

params = SLIPParams;
value = x(1) - params.l0;
isterminal = 1;
direction = 1;