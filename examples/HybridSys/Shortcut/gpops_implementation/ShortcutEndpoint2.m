%---------------------------------%
% BEGIN: ShortcutEndpoint.m %
%---------------------------------%
function output = ShortcutEndpoint2(input)

% Variables at Start and Terminus of Phase 1
t01 = input.phase(1).initialtime;
tf1 = input.phase(1).finaltime;
x01 = input.phase(1).initialstate;
xf1 = input.phase(1).finalstate;
% Variables at Start and Terminus of Phase 2
t03 = input.phase(2).initialtime;
tf3 = input.phase(2).finaltime;
x03 = input.phase(2).initialstate;
xf3 = input.phase(2).finalstate;

% Event Group 1:  Linkage Constraints Between Phases 1 and 2
output.eventgroup(1).event = [x03-xf1, t03-tf1];
% Event Group 2:  Terminus
output.eventgroup(2).event = [xf3(1)-0.8, xf3(2)+0.8];

% Objective function
objective = 0;
for i = 1 : 2
    objective = objective + input.phase(i).integral;
end

output.objective = objective;
%---------------------------------%
% BEGIN: ShortcutEndpoint.m %
%---------------------------------%
