%---------------------------------%
% BEGIN: DubinsTEndpoint.m %
%---------------------------------%
function output = DubinsLQREndpoint(input)

seq = input.auxdata.seq;
objective = 0;

for iphase = 1 : length(seq)
    
    % Event Group
    if iphase == length(seq)
        output.eventgroup(iphase).event = input.phase(iphase).finaltime;
    else
        tf1 = input.phase(iphase).finaltime;
        xf1 = input.phase(iphase).finalstate;
        t02 = input.phase(iphase+1).initialtime;
        x02 = input.phase(iphase+1).initialstate;
        
        output.eventgroup(iphase).event = [x02-xf1, t02-tf1];
    end
    
    % Objective Function
    objective = objective + input.phase(iphase).integral;
end

output.objective = objective;

%---------------------------------%
% BEGIN: DubinsTEndpoint.m %
%---------------------------------%
