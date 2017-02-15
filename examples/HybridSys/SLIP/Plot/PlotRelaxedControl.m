% Evaluate / plot the result generated by our algorithm
% Vertical partition of flight phase (preferred)
% *WITH* horizontal displacement as one of the states.
% 

current_mode = 1;
while isempty(x0{current_mode})
    current_mode = current_mode + 1;
end
xs = x0{current_mode}';

previous_mode = 0;

opt = [ ...     % 1 = actual, 0 = taylor expansion
        0;      % Dynamics
        0;      % Guard
        0;      % Reset map
       ];

MaxTime = 1;
f = cell(3,1);
g = cell(3,1);
% Dynamics
if opt(1)
    f{1} = @(xx) T * Stance_f(xx,params);
    g{1} = @(xx) T * Stance_g(xx,params);
else
    f{1} = @(xx) T * Stance_f_Approx(xx,params);
    g{1} = @(xx) T * Stance_g_Approx(xx,params);
end
f{2} = @(xx) T * Flight_f(xx,params);
g{2} = @(xx) T * Flight_g(xx,params);
f{3} = @(xx) T * Flight_f(xx,params);
g{4} = @(xx) T * Flight_g(xx,params);
% Reset map
if opt(3)
    R_12 = @(xx) Reset_S2F(xx);
    R_23 = @(xx) xx;
    R_31 = @(xx) Reset_F2S(xx);
else
    R_12 = @(xx) Reset_S2F_Approx( xx, params );
    R_23 = @(xx) xx;
    R_31 = @(xx) Reset_F2S_Approx( xx, params );
end

controller = @(tt,xx) max(0,min(1,double(subs(out.u{1}, [t;x{1}], [tt;xx]))));
% controller = @(tt,xx) 0.5;

current_time = 0;

if exist('mycolor','var') && exist('mythickness', 'var')
	P = SLIPPlot( current_mode, xs, params, mycolor, mythickness );
else
	P = SLIPPlot( current_mode, xs, params ); 
end
state_hist = [];        % [ l, ldot, theta, thetadot, x, xdot, y, ydot, mode ]
t_hist = [];

while current_time < MaxTime - 0.01
    disp(current_mode);
    switch current_mode
        case 1      % Stance
            options = odeset('Events',@(tt,xx) EvtFunc_S2F_scaled(tt,xx,params));
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
                [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{1}(xx) + g{1}(xx) * controller(tt,xx) ), ...
                             (current_time : 1e-3 : MaxTime), xs, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                if event_id(end) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    xs = R_12(xend);
                    current_mode = 2;
                end
            end
            current_time = tout(end);
            % Plot
            t_hist = [ t_hist; tout ];
            mat = [ eye(5), nan*ones(5,3) ];
            tmp = xout*mat;
            tmp(:,7) = tmp(:,1) .* cos(tmp(:,3));
            state_hist = [ state_hist; tmp, 1*ones(length(tout),1) ];
            
            P.Visualize( tout, xout, previous_mode );
            
        case 2      % Flight 1 (under ground)
            options = odeset('Events',@(tt,xx) EvtFunc_F1_scaled(tt,xx,params));
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{2}(xx) ), ...
                             (current_time : 1e-3 :  MaxTime), xs, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    xs = R_23(xend);
                    current_mode = 3;
                end
            end
            if xs(3)<yR
                xs = R_31(xs);
                current_mode = 1;
            end
            current_time = tout(end);
            % Plot
            t_hist = [t_hist; tout];
            mat = [ nan*ones(4,4), eye(4) ];
            state_hist = [ state_hist; xout*mat, 2*ones(length(tout),1) ];
            
            P.Visualize( tout, xout, previous_mode );
            
        case 3      % Flight 2 (above ground)
            options = odeset('Events',@(tt,xx) EvtFunc_F2S_scaled(tt,xx,params));
            options = odeset(options,'AbsTol',1e-9,'RelTol',1e-8);
            [ tout, xout, event_time, event_state, event_id ] = ...
                ode45(@(tt,xx) ( f{3}(xx) ), ...
                             (current_time : 1e-3 : MaxTime), xs, options);
            % Reset
            previous_mode = current_mode;
            if ~isempty(event_id)
                if event_id(1) ~= 1
                    current_mode = 0;
                else
                    xend = xout(end,:);
                    xs = R_31(xend);
                    current_mode = 1;
                end
            end
            current_time = tout(end);
            % Plot
            t_hist = [t_hist; tout];
            mat = [ nan*ones(4,4), eye(4) ];
            state_hist = [ state_hist; xout*mat, 3*ones(length(tout),1) ];
            
            P.Visualize( tout, xout, previous_mode ); 
            
        case 0
            break;
            
        otherwise
            error('Invalid Mode');
    end
end

t_hist = t_hist( state_hist(:,5)<=10 );
state_hist = state_hist( state_hist(:,5)<=10, : );
l_hist          = state_hist(:,1);
l_dot_hist      = state_hist(:,2);
theta_hist      = state_hist(:,3);
theta_dot_hist  = state_hist(:,4);
x_hist          = state_hist(:,5);
x_dot_hist      = state_hist(:,6);
y_hist          = state_hist(:,7);
y_dot_hist      = state_hist(:,8);
mode_hist       = state_hist(:,9);

%% Plot the control
u_hist = 0 * t_hist;
for i = 1 : length(t_hist)
    if ~isnan(l_hist(i))
        s = [ l_hist(i); l_dot_hist(i); theta_hist(i); theta_dot_hist(i); x_hist(i) ];
        u_hist(i) = controller( t_hist(i), s );
    else
        u_hist(i) = nan;
    end
end
% u_hist = (u_hist + 1) / 10;

figure(2);
plot(t_hist, u_hist);
xlim([0,1]);
ylim([-2,2]);

%% Key frames
% idx_mat = diff(mode_hist);
% idx = [ find( idx_mat > 0 ); find( idx_mat < 0 )+1];
% idx = sort([1; idx]);
% springcoord( [ 0, 0 ] , [ 0, params.l0 ], 5, 1.5, 0.2);
% figure;
% hold on;
% plot(x_hist, y_hist, '-.');
% for i = 1 : length(idx)
%     j = idx(i);
%     if mode_hist(j+1) == mode_hist(j)
%         xv = (x_hist(j+1) - x_hist(j)) / (t_hist(j+1) - t_hist(j));
%         yv = (y_hist(j+1) - y_hist(j)) / (t_hist(j+1) - t_hist(j));
%     elseif mode_hist(j) == mode_hist(j-1)
% %         disp(mode_hist(j));
%         xv = (x_hist(j) - x_hist(j-1)) / (t_hist(j) - t_hist(j-1));
%         yv = (y_hist(j) - y_hist(j-1)) / (t_hist(j) - t_hist(j-1));
%     end
%     PlotFrame(state_hist(j,:),params,xv,yv,gca);
% end
% axis equal
% ylim([0, 1.5]);
% xlim([0, 10]);
% set(gca, 'YTick', [0 1.5]);

%% Compute the cost
cost = 0;
for i = 1 : length(t_hist)-1
    if (t_hist(i) > 1)
        break;
    end
    if (x_hist(i) > 0)
        break;
    end
    
    current_mode = mode_hist(i);
    
    if current_mode == 1
        
        state = ( state_hist( i, 1:5 ) )';
        dt = t_hist(i+1) - t_hist(i);
        cost = cost + dt * double( subs(h{current_mode}, [ t; x{current_mode}; u{current_mode} ], [ t_hist(i); state; u_hist(i) ] ) );
       
    elseif current_mode > 1
    
        state = ( state_hist( i, 5:8 ) )';
        dt = t_hist(i+1) - t_hist(i);
        cost = cost + dt * double( subs(h{current_mode}, [ t; x{current_mode}; u{current_mode} ], [ t_hist(i); state; 0 ] ) );
        
    else
        
        disp('terminate');
        
    end
end
disp(['cost = ', num2str(cost)]);