function [value, isterminal, direction] = EvtFunc_F1_scaled( ~, x )
% Event function of Flight 1, scaled version
isterminal = ones(5,1);
direction = -ones(5,1);

value = [ x(4) + 1;     % transition to F2
          1.1 - x(1)^2;
          1.1 - x(2)^2;
          1.1 - x(3)^2;
          1.1 - x(4)^2 ];

