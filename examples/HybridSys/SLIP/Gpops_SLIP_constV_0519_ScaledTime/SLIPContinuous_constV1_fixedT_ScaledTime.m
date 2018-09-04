%-------------------------------------%
% BEGIN: SLIPContinuous_high_fixedT.m %
%-------------------------------------%
function phaseout = SLIPContinuous_constV1_fixedT(input)

nphases = input.auxdata.nphases;
params = input.auxdata.params;
T = input.auxdata.T;
v = input.auxdata.v;
phaseout = struct('dynamics',cell(1,nphases),'integrand',cell(1,nphases));

offset = input.auxdata.init;
% keyboard;
for iphase = 1 : nphases
    idx = mod(iphase+offset,3) + 1;
    t1 = input.phase(iphase).time;
    x1 = input.phase(iphase).state;
    u1 = input.phase(iphase).control;
    n1 = size(x1,1);
%     keyboard;
    switch idx
        case 1          % Stance phase
            phaseout(iphase).dynamics = T * ( Stance_f_Approx(x1', params) + Stance_g_Approx(x1', params) * u1' )';
%             phaseout(iphase).dynamics = ( Stance_f(x1') + Stance_g(x1') * u1' )';
            phaseout(iphase).integrand = 1 * ( v * T * t1 - 0.5 - x1(:,5) ).^2 * T;
%             phaseout(iphase).integrand = 0 * x1(:,1);
        case {2,3}      % Flight 1
            phaseout(iphase).dynamics = T * ( Flight_f(x1',params) )';
            phaseout(iphase).integrand = 1 * ( v * T * t1 - 0.5 - x1(:,1) ).^2 * T;
%             phaseout(iphase).integrand = 0 * x1(:,1);
        otherwise
            disp('Something Wrong!!!');
    end
    
end

%---------------------------------%
% END: SLIPContinuous.m %
%---------------------------------%