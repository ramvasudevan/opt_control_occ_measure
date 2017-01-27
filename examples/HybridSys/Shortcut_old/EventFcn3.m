function [value,isterminal,direction] = EventFcn3(~,x)

value = x(1)-1;
direction = 0;
isterminal = 1;
