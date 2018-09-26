%---------------------------------------%
% BEGIN: NatureModelEndpoint_Unscaled.m %
%---------------------------------------%
function output = NatureModelEndpoint_Unscaled(input)

nphases = input.auxdata.nphases;
params = input.auxdata.params;
l0 = input.auxdata.l0;
yR_lo = input.auxdata.yR_lo;
yR_hi = input.auxdata.yR_hi;
al = input.auxdata.alpha;
T = input.auxdata.T;
polyflag = 1;

polysin = @(ang) ang - ang.^3/6;
polycos = @(ang) 1 - ang.^2/2;

objective = 0;

% Events
for iphase = 1 : nphases-1
    idx = mod( iphase+1, 2 ) + 1;
    
    switch idx
        case 1      % Stance phase, y<=yR_hi
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
%             y = xf1(1) * polycos(xf1(3));
%             ydot = xf1(2) * polycos(xf1(3)) - xf1(1) * xf1(4) * polysin(xf1(3));
            y = xf1(1) * cos(xf1(3));
%             ydot = xf1(2) * cos(xf1(3)) - xf1(1) * xf1(4) * sin(xf1(3));
            % Linkage constraints from {y<=yR} to {y>=yR}
            % Guard: y = yR_hi
            G = y - yR_hi;
            % Reset: R
            R = xf1;
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            % The bounds should be: (0, zeros(1,4), 0) ~ (0, zeros(1,4), 0)
            
        case 2      % Stance phase, y>=yR_lo
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            y = xf1(1) * polycos(xf1(3));
%             ydot = xf1(2) * polycos(xf1(3)) - xf1(1) * xf1(4) * polysin(xf1(3));
            % Linkage constraints from flight 1 to flight 2
            % Guard: y = yR_lo
            G = y - yR_lo;
            % Reset: R
%             R = Reset_S2S( xf1', params )';
            R = Reset_S2S_poly3( xf1', params )';
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            % The bounds should be: (0, zeros(1,4), 0) ~ (0, zeros(1,4), 0)
            
            SwitchingCost = (xf1(1) * polysin(xf1(3)) + l0 * polysin(-al) - input.auxdata.d_des)^2;
            objective = objective + SwitchingCost;

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
