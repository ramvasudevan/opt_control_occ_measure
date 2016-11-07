function [value, isterminal, direction] = EvtFunc_F1( ~, x )

params = SLIPParams;
yR = params.l0 * cos(params.alpha);
value = x(3) - yR;
isterminal = 1;
direction = 1;