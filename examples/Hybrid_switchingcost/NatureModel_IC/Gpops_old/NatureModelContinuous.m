%-------------------------------------%
% BEGIN: NatureModelContinuous_high_fixedT.m %
%-------------------------------------%
function phaseout = NatureModelContinuous(input)

nphases = input.auxdata.nphases;
params = input.auxdata.params;
yR = input.auxdata.yR;
T = input.auxdata.T;
phaseout = struct('dynamics',cell(1,nphases),'integrand',cell(1,nphases));

polycos = @(ang) 1 - ang.^2/2 + ang.^4/24;

% keyboard;
for iphase = 1 : nphases
    cmode = mod(iphase,2) + 1;      % current mode
    
    t1 = input.phase(iphase).time;
    x1 = input.phase(iphase).state;
    u1 = input.phase(iphase).control;
    
    y = x1(:,1) .* polycos(x1(:,3));
%     keyboard;
    switch cmode
        case 1          % Mode 1
            phaseout(iphase).dynamics = T * ( Swing_f_poly(x1', params) + Swing_g_poly(x1', params) * u1 )';
%             phaseout(iphase).dynamics = ( Stance_f(x1') + Stance_g(x1') * u1' )';
            phaseout(iphase).integrand = u1 .^ 2;
            phaseout(iphase).path = y;
            
        case 2      	% Mode 2
            phaseout(iphase).dynamics = T * ( Swing_f_poly(x1', params) + Swing_g_poly(x1', params) * u1 )';
            phaseout(iphase).integrand = u1 .^ 2;
            phaseout(iphase).path = y;
        otherwise
            disp('Something Wrong!!!');
    end
    
end

%---------------------------------%
% END: NatureModelContinuous.m %
%---------------------------------%