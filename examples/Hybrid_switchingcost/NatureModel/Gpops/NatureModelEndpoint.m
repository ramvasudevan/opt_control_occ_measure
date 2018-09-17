%-----------------------------------%
% BEGIN: NatureModelEndpoint_high_fixedT.m %
%-----------------------------------%
function output = NatureModelEndpoint(input)

nphases = input.auxdata.nphases;
params = input.auxdata.params;
l0 = input.auxdata.l0;
yR = input.auxdata.yR;
al = params.alpha;
T = input.auxdata.T;
polyflag = 1;

polysin = @(ang) ang - ang.^3/6 + ang.^5/120;
polycos = @(ang) 1 - ang.^2/2 + ang.^4/24;

% Events
for iphase = 1 : nphases-1
    cmode = mod(iphase,2) + 1;      % current mode
    
    switch cmode
        case 1      % Mode 1
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            % Linkage constraints from Mode 1 to Mode 2
            y = xf1(1) * polycos(xf1(3));
            ydot = xf1(2) * polycos(xf1(3)) - xf1(1) * xf1(4) * polysin(xf1(3));
            % Guard: y = yR, ydot > 0
            G = [ y, ydot ];
            % Reset: identity
            R = xf1;
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            % The bounds should be: (0, zeros(1,4), yR, 0) ~ (0, zeros(1,4), yR, ldotmax)
            
        case 2      % Mode 2
            tf1 = input.phase(iphase).finaltime;
            xf1 = input.phase(iphase).finalstate;
            t02 = input.phase(iphase+1).initialtime;
            x02 = input.phase(iphase+1).initialstate;
            
            % Linkage constraints from Mode 2 to Mode 1
            y = xf1(1) * polycos(xf1(3));
            ydot = xf1(2) * polycos(xf1(3)) - xf1(1) * xf1(4) * polysin(xf1(3));
            % Guard: y = yR, ydot < 0
            G = [ y, ydot ];
            % Reset R
            if polyflag
                R = ( Reset_S2S_poly( xf1', params ) )';
            else
                R = ( Reset_S2S( xf1', params ) )';
            end
            output.eventgroup(iphase).event = [ t02 - tf1, x02 - R, G ];
            
            % The bounds should be: (0, zeros(1,4), yR, -ldotmax) ~ (0, zeros(1,4), yR, 0)
    end
end

% Terminal condition
iphase = nphases;
tf1 = input.phase(iphase).finaltime;
output.eventgroup(iphase).event = [ tf1 - T ];

% Objective function
objective = 0;
for i = 1 : nphases
    xf1 = input.phase(iphase).finalstate;
    if (cmode == 2) && (i ~= nphases)
        SwitchingCost = (xf1(1) * polysin(xf1(3)) + l0 * sin(-al) - input.auxdata.d_des)^2;
        objective = objective + SwitchingCost;
    end
    objective = objective + input.phase(i).integral;
end

output.objective = objective;

%---------------------------------%
% END: NatureModelEndpoint.m %
%---------------------------------%
