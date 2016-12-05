function [value, isterminal, direction] = EvtFunc_S2F_scaled( ~, x )
% Event function of Stance 1, scaled version
isterminal = ones(6,1);
direction = -ones(6,1);

value = [ 1 - x(1);     % transition to Flight 1
          1.2 - x(1)^2;
          1.2 - x(2)^2;
          1.2 - x(3)^2;
          1.2 - x(4)^2;
          1.2 - x(5)^2 ];
