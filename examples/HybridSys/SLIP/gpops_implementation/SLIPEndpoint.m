%---------------------------------%
% BEGIN: SLIPEndpoint.m %
%---------------------------------%
function output = SLIPEndpoint(input)

nphases = input.auxdata.nphases;
l0 = input.auxdata.l0;
yR = input.auxdata.yR;

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
            G = [ xf1(1) - l0, xf1(2) ];
            % Reset: R
%             R = ( Reset_S2F_Approx( xf1' ) )';
            R = ( Reset_S2F( xf1' ) )';
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            % The bounds should be: (0, zeros(1,4), 0, 0) ~ (0, zeros(1,4), 0, ldotmax)
            
        case 2
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            % Linkage constraints from flight 1 to flight 2
            % Guard: ydot = 0
            G = xf1(4);
            % Reset: identity
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - xf1, G ];
            
            % The bounds should be: (0, zeros(1,4), 0) ~ itself
            
        case 3
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            % Linkage constraints from flight 2 to stance (Touch-down)
            % Guard: y = yR
            G = xf1(3) - yR;
            % Reset: R
%             R = ( Reset_F2S_Approx( xf1' ) )';
            R = ( Reset_F2S( xf1' ) )';
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
output.eventgroup(iphase).event = [ xf1(1) - 2.5 ];
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
