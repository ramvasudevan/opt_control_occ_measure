%-----------------------------------%
% BEGIN: DubinsTContinuous.m %
%-----------------------------------%
function phaseout = DubinsTContinuous(input)

%---------------------%
% Dynamics in Phase 1 %
%---------------------%
x1 = input.phase(1).state;
u1 = input.phase(1).control;
n1 = size(x1,1);

xdot = u1(:,1) .* cos(x1(:,3));
ydot = u1(:,1) .* sin(x1(:,3));
thetadot = u1(:,2);
phaseout(1).dynamics = [ xdot, ydot, thetadot ];
phaseout(1).integrand = ones(n1,1);

%---------------------%
% Dynamics in Phase 2 %
%---------------------%
x2 = input.phase(2).state;
u2 = input.phase(2).control;
n2 = size(x2,1);

xdot = u2(:,1) .* cos(x2(:,3));
ydot = u2(:,1) .* sin(x2(:,3));
thetadot = u2(:,2);
phaseout(2).dynamics = [ xdot, ydot, thetadot ];
phaseout(2).integrand = ones(n2,1);

%---------------------%
% Dynamics in Phase 3 %
%---------------------%
x3 = input.phase(3).state;
u3 = input.phase(3).control;
n3 = size(x3,1);

xdot = u3(:,1) .* cos(x3(:,3));
ydot = u3(:,1) .* sin(x3(:,3));
thetadot = u3(:,2);
phaseout(3).dynamics = [ xdot, ydot, thetadot ];
phaseout(3).integrand = ones(n3,1);

%---------------------------------%
% END: DubinsTContinuous.m %
%---------------------------------%
