%-----------------------------------%
% BEGIN: SLIPEndpoint_high_fixedT.m %
%-----------------------------------%
function output = SLIPEndpoint_far_fixedT(input)

nphases = input.auxdata.nphases;
l0 = input.auxdata.l0;
yR = input.auxdata.yR;
offset = input.auxdata.init;
T = input.auxdata.T;
polyflag = 1;

% Events
for iphase = 1 : nphases-1
    idx = mod(iphase+offset,3) + 1;
    
    switch idx
        case 1      % Stance phase
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            % Linkage constraints from stance to flight 1 (Take-off)
            % Guard: l = lmax, ldot >= 0
            G = [ xf1(1) - l0, xf1(2) ];
            % Reset: R
            if polyflag
                R = ( Reset_S2F_Approx( xf1' ) )';
            else
                R = ( Reset_S2F( xf1' ) )';
            end
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            % The bounds should be: (0, zeros(1,4), 0, 0) ~ (0, zeros(1,4), 0, ldotmax)
            
        case 2      % Flight phase 1
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
            
        case 3      % Flight phase 2
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            % Linkage constraints from flight 2 to stance (Touch-down)
            % Guard: y = yR
            G = xf1(3) - yR;
            % Reset: R
            if polyflag
                R = ( Reset_F2S_Approx( xf1' ) )';
            else
                R = ( Reset_F2S( xf1' ) )';
            end
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            % The bounds should be: (0, zeros(1,5), 0) ~ itself
            
    end
end

% Terminal condition
iphase = nphases;
tf1 = input.phase(iphase).finaltime;
output.eventgroup(iphase).event = [ tf1 - T ];

% Objective function
iphase = nphases;
idx = mod(iphase+offset,3) + 1;
state = input.phase(iphase).finalstate;
switch idx
    case 1
        objective = state(5);
    case {2,3}
        objective = state(1);
end

output.objective = objective;

%---------------------------------%
% END: SLIPEndpoint.m %
%---------------------------------%
