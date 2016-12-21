function [value, isterminal, direction] = EvtFunc_F2S_scaled( ~, x )
% Event function of Flight 2, scaled version
isterminal = ones(8,1);
direction = -ones(8,1);
% direction = zeros(8,1);
params = SLIPParams;
domain = params.domain_size{3};
yR = params.yR;

value = [ x(3) - yR;                    % y = yR
          domain(1:2,2) - x(1:2);
          x(1:2) - domain(1:2,1);
          domain(3,2) - x(3);
          domain(4,2) - x(4);
          x(4) - domain(4,1) ];
if value(1) < 0
    value(1) = 0;
end