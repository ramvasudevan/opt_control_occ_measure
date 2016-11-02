function [value, isterminal, direction] = EvtFunc_F2S( ~, x )

params = SLIPParams;
value(1) = x(3) - params.yR;
value(2) = x(3) - params.yR;
isterminal(1) = 0;
isterminal(2) = 1;
direction(1) = 1;
direction(2) = -1;