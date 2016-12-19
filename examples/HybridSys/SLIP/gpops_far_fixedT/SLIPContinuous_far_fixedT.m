%-------------------------------------%
% BEGIN: SLIPContinuous_high_fixedT.m %
%-------------------------------------%
function phaseout = SLIPContinuous_far_fixedT(input)

nphases = input.auxdata.nphases;
phaseout = struct('dynamics',cell(1,nphases),'integrand',cell(1,nphases));

offset = input.auxdata.init;

for iphase = 1 : nphases
    idx = mod(iphase+offset,3) + 1;
    x1 = input.phase(iphase).state;
    u1 = input.phase(iphase).control;
    n1 = size(x1,1);
    
    switch idx
        case 1          % Stance phase
%             phaseout(iphase).dynamics = ( Stance_f_Approx(x1') + Stance_g_Approx(x1') * u1' )';
            phaseout(iphase).dynamics = ( Stance_f(x1') + Stance_g(x1') * u1' )';
            phaseout(iphase).integrand = x1(:,1) .* cos(x1(:,3));
        case {2,3}      % Flight 1
            phaseout(iphase).dynamics = ( Flight_f(x1') )';
            phaseout(iphase).integrand = x1(:,3);
        otherwise
            disp('Something Wrong!!!');
    end
    
end

%---------------------------------%
% END: SLIPContinuous.m %
%---------------------------------%