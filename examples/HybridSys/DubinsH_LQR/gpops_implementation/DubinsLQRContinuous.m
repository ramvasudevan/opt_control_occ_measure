%-----------------------------------%
% BEGIN: DubinsTContinuous.m %
%-----------------------------------%
function phaseout = DubinsLQRContinuous(input)

% xref = [ 0.5, -0.2, 0 ];
seq = input.auxdata.seq;

for iphase = 1 : length(seq)
    x1 = input.phase(iphase).state;
    u1 = input.phase(iphase).control;
    
    % xdot = u1(:,1) .* cos(x1(:,3));
    % ydot = u1(:,1) .* sin(x1(:,3));
    xdot = u1(:,1) .* ( 1 - x1( :,3 ).^2/2 );
    ydot = u1(:,1) .* x1(:,3);
    thetadot = u1(:,2);
    phaseout(iphase).dynamics = [ xdot, ydot, thetadot ];
    phaseout(iphase).integrand = ...
    (x1(:,1) - 0.5).^2 + (x1(:,2) + 0.2).^2 + u1(:,1).^2 + (u1(:,2)/3).^2;
end

%---------------------------------%
% END: DubinsTContinuous.m %
%---------------------------------%
