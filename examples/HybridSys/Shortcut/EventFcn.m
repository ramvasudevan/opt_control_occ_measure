function [value,isterminal,direction] = EventFcn(~,x)

value(1) = x(2) - 1;
direction(1) = 0;
isterminal(1) = 1;

value(2) = x(1) - 0.8;
direction(2) = 0;
isterminal(2) = 1;