function [value,isterminal,direction] = EventFcn(~,x)

value(1) = x(1) - 1;
value(2) = x(1) + 1;
value(3) = x(2) - 1;
value(4) = x(2) + 1;

direction = zeros(1,4);
isterminal = ones(1,4);
