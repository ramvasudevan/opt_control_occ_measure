function [value, isterminal, direction] = EvtFunc_F2S_Approx( ~, x )
% This function is exactly the same as EvtFunc_F2S.m, i.e.
% we don't need the approximation of cos(alpha) in this case.
polycos = @(x) cos(x);
params = SLIPParams;
yR = params.l0 * polycos(params.alpha);
value(1) = x(3) - yR;
value(2) = x(3) - yR;
isterminal(1) = 0;
isterminal(2) = 1;
direction(1) = 1;
direction(2) = -1;