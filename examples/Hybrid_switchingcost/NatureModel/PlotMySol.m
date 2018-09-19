% Simulate forward Nature model
% walking (no flight phase)

% function [tval, xval] = SimNatureModel_walking( params, x0 )
close all

MaxTime = 1;
current_mode = 2;
while isempty(x0{current_mode})
    current_mode = current_mode + 1;
end
xs = x0{current_mode}';

previous_mode = 0;

polysin = @(ang) ang - ang.^3/6 + ang.^5/120;
polycos = @(ang) 1 - ang.^2/2 + ang.^4/24;

% Dynamics
Dyn         = cell(2,1);
controller  = cell(2,1);

controller{1} = @(tt,xx) max(0,min(umax,double(subs(out.u{1}, [t;x{1}], [tt;xx]))));
controller{2} = @(tt,xx) max(0,min(umax,double(subs(out.u{2}, [t;x{1}], [tt;xx]))));
% controller{1} = @(tt,xx) max(0, 1 * (params.l0 - xx(1) ));
% controller{2} = @(tt,xx) max(0, 1 * (params.l0 - xx(1) ));
% controller{1} = @(tt,xx) 0;
% controller{2} = @(tt,xx) 0;

Dyn{1} = @(tt,xx) T * ( Swing_f_poly(xx,params) + Swing_g_poly(xx,params) * controller{1}(tt,xx) );
Dyn{2} = @(tt,xx) T * ( Swing_f_poly(xx,params) + Swing_g_poly(xx,params) * controller{2}(tt,xx) );
% Dyn{1} = @(tt,xx) T * ( Swing_f(xx,params) + Swing_g_poly(xx,params) * controller{1}(tt,xx) );
% Dyn{2} = @(tt,xx) T * ( Swing_f(xx,params) + Swing_g_poly(xx,params) * controller{2}(tt,xx) );

% Reset maps
ResetMap = cell(2,2);
ResetMap{1,2} = @(xx) xx;
ResetMap{2,1} = @(xx) Reset_S2S_poly( xx, params );

% Simulate af!
state_hist = [];        % [ l, ldot, theta, thetadot ]
t_hist = [];
origin_hist = [];
mode_hist = [];
phase_hist = [];
previous_origin = 0;
current_time = 0;

P = NatureModelPlot( current_mode, xs, params );
current_phase = 1;

while current_time < MaxTime - 0.01
    disp(current_mode);
    switch current_mode
        case 1
            options = odeset('Events',@(tt,xx) EvtFunc12(tt,xx,params));
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(Dyn{1}, (current_time : 1e-3 : MaxTime), xs, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                if event_id(end) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    xs = ResetMap{1,2}(xend');
                    current_mode = 2;
                end
            end
            current_time = tout(end);
            
            t_hist = [ t_hist; tout ];
            state_hist = [ state_hist; xout ];
            
            P.Visualize( tout, xout, previous_mode );
            
            origin_hist = [ origin_hist; repmat(previous_origin, length(tout), 1)];
            mode_hist = [ mode_hist; ones( length(tout), 1 ) ];
            phase_hist = [ phase_hist; ones( length(tout), 1 ) * current_phase ];
            current_phase = current_phase + 1;
        case 2
            options = odeset('Events',@(tt,xx) EvtFunc21(tt,xx,params));
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(Dyn{2}, (current_time : 1e-3 : MaxTime), xs, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                if event_id(end) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    xs = ResetMap{2,1}(xend');
                    current_mode = 1;
                end
            end
            current_time = tout(end);
            
            t_hist = [ t_hist; tout ];
            state_hist = [ state_hist; xout ];
            
            P.Visualize( tout, xout, previous_mode );
            
            origin_hist = [ origin_hist; repmat(previous_origin, length(tout), 1)];
            previous_origin = previous_origin + state_hist(end,1) * polysin(state_hist(end,3)) + params.l0 * sin(-params.alpha);
            mode_hist = [ mode_hist; 2 * ones( length(tout), 1 ) ];
            phase_hist = [ phase_hist; ones( length(tout), 1 ) * current_phase ];
            current_phase = current_phase + 1;
        case 0
            break;
        otherwise
            error('Invalid Mode');
    end
end

%% plot states
l_hist = state_hist( :, 1 );
theta_hist = state_hist( :, 3 );
x_hist = origin_hist + l_hist .* polysin( theta_hist );
y_hist = l_hist .* polycos( theta_hist );
u_hist = zeros( length(t_hist), 1 );
for i = 1 : length(t_hist)
    u_hist(i) = controller{mode_hist(i)}(t_hist(i), state_hist(i,:)');
end

% Figure 2 shows x-y
figure(2);
hold on;
title('x-y');
plot( x_hist, y_hist );
plot( origin_hist, origin_hist * 0, 'ro' );

% figure 3 shows states
figure(3);
hold on;
title('states');
plot( t_hist, state_hist );
plot( t_hist, u_hist );
plot( t_hist, phase_hist );
legend('l', 'ldot', 'theta', 'thetadot', 'u', 'phase');


%% Generate initial guess
nphases = 1;
T_new = 0.8;
len = nnz(t_hist < T_new/T);
t_hist = t_hist( t_hist < T_new/T );
for iphase = 1 : nphases
    idx = find( phase_hist(1:len) == iphase );
    myguess.phase(iphase).time = t_hist( idx );
    myguess.phase(iphase).state = state_hist( idx, : );
    myguess.phase(iphase).control = zeros( length(idx), 1 );
    for cnt = 1 : length(idx)
        cmode = mod(iphase,2)+1;
        myguess.phase(iphase).control(cnt) = controller{cmode}(t_hist( idx(cnt) ), state_hist( idx(cnt),: )');
    end
    
    myguess.phase(iphase).time = t_hist( idx ) / 0.55;
%     guess.phase(iphase).time = [dt*(iphase-1); dt*iphase ];
%     guess.phase(iphase).state = zeros( 2, 4 );
%     guess.phase(iphase).control = [0; 0];
    myguess.phase(iphase).integral = 0;
end