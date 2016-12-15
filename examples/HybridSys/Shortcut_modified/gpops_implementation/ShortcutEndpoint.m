%---------------------------------%
% BEGIN: ShortcutEndpoint.m %
%---------------------------------%
function output = ShortcutEndpoint(input)

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
R12 = -xf1(1).^2 + 0.5;
output.eventgroup(1).event = [xf1(2)-1, x02-R12, t02-tf1];
% Event Group 2:  Linkage Constraints Between Phases 2 and 3
R23 = [0.6, -0.8, 0];
output.eventgroup(2).event = [xf2-1, x03-R23, t03-tf2];
% Event Group 3:  Constraints on Terminal State
output.eventgroup(3).event = [xf3(1)-0.8, xf3(2)+0.8];

% Objective function
objective = 0;
for i = 1 : 3
    objective = objective + input.phase(i).integral;
end

output.objective = objective;
%---------------------------------%
% BEGIN: ShortcutEndpoint.m %
%---------------------------------%
