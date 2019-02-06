% SLIP model with 3 modes.
% Goal: follow a constant-speed trajectory during t \in [0,T]
% running cost = ((v*t - 0.5) - x)^2, where v  = 0.1
% terminal cost = 0
% 

clear;
load('Rebuttal_SLIP_high_d6_T2.5minimum.mat');
% load('Rebuttal_SLIP_constV_d6_T3minimum.mat');

%-------------------------------------------------------------------------%
%-------------------------------- Solve ----------------------------------%
%-------------------------------------------------------------------------%
P = SLIPPlot( 3, x0{3}, params ); 


% function handles
hf{1} = @(xx) Stance_f_Approx(xx,params);
hg{1} = @(xx) Stance_g_Approx(xx,params);
hf{2} = @(xx) Flight_f(xx,params);
hg{2} = @(xx) Flight_g(xx,params);
hf{3} = @(xx) Flight_f(xx,params);
hg{4} = @(xx) Flight_g(xx,params);

R_12 = @(xx) Reset_S2F_Approx( xx, params );
R_23 = @(xx) xx;
R_31 = @(xx) Reset_F2S_Approx( xx, params );

MaxTime = 1;

t_hist = 0;
state_hist = nan(1,9);
u_hist = 0;
pval = 0;
total_time = 0;

seq = [3 1 2 3];

for i = 1 : length( seq )
%     Tleft = T - t_hist(end);
%     Tleft = T;
    
    cmode = seq( i );
    xvar = x{ cmode };
    uvar = u{ cmode };
    if i == 1
        cx0 = x0{ cmode };
    end
    
    
    MaxTime = (T - t_hist(end)) / T;
    % integrate forward
    controller = @(tt,xx) max(0,min(1,double(subs(out.u{i}, [t;xvar], [tt;xx]))));
    switch cmode
        case 1          % stance
            ode_options = odeset('Events',@(tt,xx) EvtFunc_S2F_scaled(tt,xx,params));
            ode_options = odeset(ode_options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) T * ( hf{1}(xx) + hg{1}(xx) * controller(tt,xx) ), ...
                             (0 : 1e-3 : MaxTime), cx0, ode_options);
            % Reset
            if ~isempty(event_id)
                if event_id(end) ~= 1
                    error('Something wrong.');
                else
                    xend = xout(end,:);
                    cx0 = R_12(xend);
                end
            end
            % Save
            P.Visualize( tout * T + t_hist(end), xout, 1 );
            
            t_hist = [ t_hist; t_hist(end) + tout * T ];
            mat = [ eye(5), nan*ones(5,3) ];
            tmp = xout*mat;
            tmp(:,7) = tmp(:,1) .* cos(tmp(:,3));
            state_hist = [ state_hist; tmp, 1*ones(length(tout),1) ];
        case 2          % flight 1, ydot > 0
            ode_options = odeset('Events',@(tt,xx) EvtFunc_F1_scaled(tt,xx,params));
            ode_options = odeset(ode_options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) T * ( hf{2}(xx) ), ...
                             (0 : 1e-3 :  MaxTime), cx0, ode_options);
            % Reset
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    error('Something wrong 2.');
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    cx0 = R_23(xend);
                    cx0 = cx0(:);
                    current_mode = 3;
                end
            end
%             if xs(3)<yR
%                 xs = R_31(xs);
%                 current_mode = 1;
%             end
%             current_time = tout(end);
            % Plot
            P.Visualize( tout * T + t_hist(end), xout, 2 );
            t_hist = [t_hist; t_hist(end) + tout * T];
            mat = [ nan*ones(4,4), eye(4) ];
            state_hist = [ state_hist; xout*mat, 2*ones(length(tout),1) ];
            
%             P.Visualize( tout, xout, 2 );
        case 3          % flight 2, ydot < 0
            ode_options = odeset('Events',@(tt,xx) EvtFunc_F2S_scaled(tt,xx,params));
            ode_options = odeset(ode_options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) T * ( hf{3}(xx) ), ...
                             (0 : 1e-3 : MaxTime), cx0, ode_options);
            % Reset
%             previous_mode = current_mode;
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    current_mode = 0;
                    error('Something wrong 3.');
                else
                    xend = xout(end,:);
                    cx0 = R_31(xend);
                    current_mode = 1;
                end
            end
            current_time = tout(end);
            % Plot
            P.Visualize( tout * T + t_hist(end), xout, 3 );
            
            t_hist = [t_hist; t_hist(end) + tout * T];
            mat = [ nan*ones(4,4), eye(4) ];
            state_hist = [ state_hist; xout*mat, 3*ones(length(tout),1) ];
            
%             P.Visualize( tout, xout, 3 );
    end
    
end


%% -------------------------------------------------------------------------%
%-------------------------------- Cost -----------------------------------%
%-------------------------------------------------------------------------%
t_hist = t_hist( 2:end );
state_hist = state_hist( 2:end, : );


cost = 0;
for i = 1 : length(t_hist)-1
    
    current_mode = state_hist(i,end);
    
    if current_mode == 1
        
        state = ( state_hist( i, 1:5 ) )';
        dt = t_hist(i+1) - t_hist(i);
        cost = cost + dt * double( subs(h{current_mode}, [ t; x{current_mode} ], [ t_hist(i); state ] ) ) / T;
       
    elseif current_mode > 1
    
        state = ( state_hist( i, 5:8 ) )';
        dt = t_hist(i+1) - t_hist(i);
        cost = cost + dt * double( subs(h{current_mode}, [ t; x{current_mode} ], [ t_hist(i); state ] ) ) / T;
        
    else
        
        disp('terminate');
        
    end
end

disp(['cost = ', num2str(cost)]);

