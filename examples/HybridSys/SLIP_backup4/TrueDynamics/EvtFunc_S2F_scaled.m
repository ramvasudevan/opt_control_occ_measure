function [value, isterminal, direction] = EvtFunc_S2F_scaled( ~, x )
% Event function of Stance 1, scaled version
isterminal = ones(10,1);
direction = -ones(10,1);
params = SLIPParams;
l0 = params.l0;
domain = params.domain_size{1};

value = [ l0 - x(1);     % transition to Flight 1
          x(1) - domain(1,1);
          domain(2:end,2) - x(2:end) + 0.5;
          x(2:end) - domain(2:end,1) + 0.5];
