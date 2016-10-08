function [value,isterminal,direction] = EventFcn(~,x)

value = x(2) - 1;
direction = 0;
isterminal = 1;