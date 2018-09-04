function dydt = VDP_Dyn( var )
x = var(1) * 2.5;
y = var(2) * 3;
mu = 0.3;
dydt = [ y / 2.5;
         (mu*(1-x^2)*y - x) / 3 ];

% x = var(1);
% y = var(2);
% mu = 0.3;
% dydt = [ y;
%          (mu*(1-x^2)*y - x) ];
% x = var(1);
% y = var(2);
% mu = 0.5;
% dydt = [ y;
%          (mu * (1-x^2)*y - x) ];

% s = 1;
% dydt = [ var(2)/ s; -var(1)*s ];