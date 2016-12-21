%-----------------------------------%
% BEGIN: SLIPContinuous.m %
%-----------------------------------%
function phaseout = SLIPContinuous_scaled(input)
% keyboard
nphases = input.auxdata.nphases;
phaseout = struct('dynamics',cell(1,nphases),'integrand',cell(1,nphases));

domain_size = input.auxdata.params.domain_size;
for i = 1 : 3
    scale_x{i} = (domain_size{i}(:,2) - domain_size{i}(:,1)) / 2;
    trans_x{i} = mean(domain_size{i},2);
end


for iphase = 1 : nphases
    idx = mod(iphase+1,3) + 1;
    x1 = input.phase(iphase).state;
    u1 = input.phase(iphase).control;
    n1 = size(x1,1);
    
    switch idx
        case 1          % Stance phase
%             phaseout(iphase).dynamics = ( Stance_f_Approx(x1') + Stance_g_Approx(x1') * u1' )';
%             phaseout(iphase).dynamics = ( Stance_f(x1') + Stance_g_Approx(x1') * u1' )';
            phaseout(iphase).dynamics = ...
                (rescale_dynamics_gpops(@Stance_f_Approx, x1, scale_x{idx}, trans_x{idx}, scale_x{idx} ) + ...
                    u1 * (Stance_g_Approx(x1))');
            phaseout(iphase).integrand = ones(n1,1);
        case {2,3}      % Flight 1
%             phaseout(iphase).dynamics = ( Flight_f(x1') )';
            phaseout(iphase).dynamics = ...
                (rescale_dynamics_gpops(@Flight_f, x1, scale_x{idx}, trans_x{idx}, scale_x{idx}));
            phaseout(iphase).integrand = ones(n1,1);
        otherwise
            disp('Something Wrong!!!');
    end
    
end

%---------------------------------%
% END: SLIPContinuous.m %
%---------------------------------%