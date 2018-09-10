% Simulate forward Nature model
% walking (no flight phase)

function [tval, xval] = SimNatureModel_walking( params, x0,  )

current_mode = 1;
while isempty(x0{current_mode})
    current_mode = current_mode + 1;
end
xs = x0{current_mode}';

previous_mode = 0;.

% Dynamics
Dyn         = cell(2,1);
controller  = cell(2,1);

controller{1} = @(tt,xx) max(0,min(umax,double(subs(out.u{1}, [t;x{1}], [tt;xx]))));
controller{2} = @(tt,xx) max(0,min(umax,double(subs(out.u{2}, [t;x{1}], [tt;xx]))));

Dyn{1} = @(tt,xx) T * ( Swing_f_poly(xx,params) + Swing_g_poly{1}(xx) * controller(tt,xx) );
Dyn{2} = Dyn{1};


