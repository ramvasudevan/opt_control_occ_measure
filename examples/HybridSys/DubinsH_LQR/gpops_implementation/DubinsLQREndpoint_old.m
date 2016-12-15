%---------------------------------%
% BEGIN: DubinsTEndpoint.m %
%---------------------------------%
function output = DubinsLQREndpoint_old(input)

% Variables at Start and Terminus of Phase 1
t01 = input.phase(1).initialtime;
tf1 = input.phase(1).finaltime;
x01 = input.phase(1).initialstate;
xf1 = input.phase(1).finalstate;
% Variables at Start and Terminus of Phase 2
t02 = input.phase(2).initialtime;
tf2 = input.phase(2).finaltime;
x02 = input.phase(2).initialstate;
xf2 = input.phase(2).finalstate;
% Variables at Start and Terminus of Phase 3
t03 = input.phase(3).initialtime;
tf3 = input.phase(3).finaltime;
x03 = input.phase(3).initialstate;
xf3 = input.phase(3).finalstate;

% Event Group 1:  Linkage Constraints Between Phases 1 and 2
output.eventgroup(1).event = [x02-xf1, t02-tf1];
% Event Group 2:  Linkage Constraints Between Phases 2 and 3
output.eventgroup(2).event = [x03-xf2, t03-tf2];
% Event Group 3:  Constraints on Terminal State
output.eventgroup(3).event = tf3;


% Objective function
objective = 0;
for i = 1 : 3
    objective = objective + input.phase(i).integral;
end

output.objective = objective;
%---------------------------------%
% BEGIN: DubinsTEndpoint.m %
%---------------------------------%
