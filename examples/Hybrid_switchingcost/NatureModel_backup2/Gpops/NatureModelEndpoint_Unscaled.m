%---------------------------------------%
% BEGIN: NatureModelEndpoint_Unscaled.m %
%---------------------------------------%
function output = NatureModelEndpoint_Unscaled(input)

nphases = input.auxdata.nphases;
params = input.auxdata.params;
l0 = input.auxdata.l0;
yR = input.auxdata.yR;
al = input.auxdata.alpha;
T = input.auxdata.T;
polyflag = 1;

polysin = @(ang) ang - ang.^3/6 + ang.^5/120;
polycos = @(ang) 1 - ang.^2/2 + ang.^4/24;

objective = 0;

% Events
for iphase = 1 : nphases-1
    idx = mod( iphase, 2 ) + 1;
    
    switch idx
        case 1      % Stance phase, y<=yR
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
%             y = xf1(1) * polycos(xf1(3));
%             ydot = xf1(2) * polycos(xf1(3)) - xf1(1) * xf1(4) * polysin(xf1(3));
            y = xf1(1) * cos(xf1(3));
            ydot = xf1(2) * cos(xf1(3)) - xf1(1) * xf1(4) * sin(xf1(3));
            % Linkage constraints from {y<=yR} to {y>=yR}
            % Guard: y = yR, ydot >= 0
            G = [ y - yR, ydot ];
            % Reset: R
            R = xf1;
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            % The bounds should be: (0, zeros(1,4), 0, 0) ~ (0, zeros(1,4), 0, ydotmax)
            
        case 2      % Stance phase, y>=yR
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            y = xf1(1) * polycos(xf1(3));
            ydot = xf1(2) * polycos(xf1(3)) - xf1(1) * xf1(4) * polysin(xf1(3));
            % Linkage constraints from flight 1 to flight 2
            % Guard: y = yR, ydot <= 0
            G = [ y - yR, ydot ];
            % Reset: R
%             R = Reset_S2S( xf1', params )';
            R = Reset_S2S_poly( xf1', params )';
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            
            SwitchingCost = (xf1(1) * polysin(xf1(3)) + l0 * sin(-al) - input.auxdata.d_des)^2;
%             objective = objective + SwitchingCost;
            % The bounds should be: (0, zeros(1,4), 0, -ydotmax) ~ (0, zeros(1,4), 0, 0)
    end
end

% Terminal condition
iphase = nphases;
tf1 = input.phase(iphase).finaltime;
output.eventgroup(iphase).event = [ tf1 - T ];

% Objective function
for i = 1 : nphases
    objective = objective + input.phase(i).integral;
end

output.objective = objective;

%---------------------------------%
% END: SLIPEndpoint.m %
%---------------------------------%
