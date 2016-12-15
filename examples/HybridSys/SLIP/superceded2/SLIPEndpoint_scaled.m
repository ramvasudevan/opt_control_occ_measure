%---------------------------------%
% BEGIN: SLIPEndpoint.m %
%---------------------------------%
function output = SLIPEndpoint_scaled(input)
% keyboard
nphases = input.auxdata.nphases;
l0 = input.auxdata.l0;
yR = input.auxdata.yR;

domain_size = input.auxdata.params.domain_size;
for i = 1 : 3
    scale_x{i} = (domain_size{i}(:,2) - domain_size{i}(:,1)) / 2;
    trans_x{i} = mean(domain_size{i},2);
end

polyflag = 0;

% Events
for iphase = 1 : nphases-1
    idx = mod(iphase+1,3) + 1;
    
    switch idx
        case 1
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            % Linkage constraints from stance to flight 1 (Take-off)
            % Guard: l = lmax, ldot >= 0
            G = [ xf1(1) - 1, xf1(2) ];
            % Reset: R
            if polyflag
                R = rescale_reset(@Reset_S2F_Approx, xf1', scale_x{1}, trans_x{1}, scale_x{2}, trans_x{2});
                R = R';
            else
                R = rescale_reset(@Reset_S2F, xf1', scale_x{1}, trans_x{1}, scale_x{2}, trans_x{2});
                R = R';
            end
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            % The bounds should be: (0, zeros(1,4), 0, 0) ~ (0, zeros(1,4), 0, ldotmax)
            
        case 2
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            % Linkage constraints from flight 1 to flight 2
            % Guard: ydot = 0
            G = xf1(4)+1;
            % Reset: identity
            R = rescale_reset(@(xx) xx, xf1', scale_x{2}, trans_x{2}, scale_x{3}, trans_x{3});
            R = R';
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            % The bounds should be: (0, zeros(1,4), 0) ~ itself
            
        case 3
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            % Linkage constraints from flight 2 to stance (Touch-down)
            % Guard: y = yR
            G = xf1(3) + 1;
            % Reset: R
            if polyflag
                R = rescale_reset(@Reset_F2S_Approx, xf1', scale_x{3}, trans_x{3}, scale_x{1}, trans_x{1});
                R = R';
            else
                R = rescale_reset(@Reset_F2S, xf1', scale_x{3}, trans_x{3}, scale_x{1}, trans_x{1});
                R = R';
            end
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            % The bounds should be: (0, zeros(1,5), 0) ~ itself
            
    end
end

% Terminal condition
iphase = nphases;
xf1 = input.phase(iphase).finalstate;
% x >= 8
% ------------------ Warning !!!! ------------------
% May be xf1(1) or xf1(5). Remember to check this!!!
% --------------------------------------------------
TarPt = rescale_guard_gpops(@(xx) xx(1)-2.5, xf1, scale_x{3}, trans_x{3});
output.eventgroup(iphase).event = TarPt;
% The bounds should be: 0~2 (or, maybe, 0~1000)


% Objective function
objective = 0;
for i = 1 : nphases
    objective = objective + input.phase(i).integral;
end

output.objective = objective;

%---------------------------------%
% END: SLIPEndpoint.m %
%---------------------------------%
