function [value, isterminal, direction] = EvtFunc_F2S_scaled( ~, x )
% Event function of Flight 2, scaled version
isterminal = ones(5,1);
direction = -ones(5,1);

value = [ x(3) + 1;     % transition to stance phase
          1.1 - x(1)^2;
          1.1 - x(2)^2
          1.1 - x(3)^2;
          1.1 - x(4)^2 ];