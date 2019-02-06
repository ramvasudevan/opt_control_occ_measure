
if exist('Rebuttal_GPOPS_constV.mat','file')
    load('Rebuttal_GPOPS_constV.mat');
end

if ~exist('info', 'var')
    cnt = 1;
    info = struct();
else
    cnt = length(info)+1;
end

for nphases = 2:2
    clc;
    disp(['SLIP, high, nphases = ', num2str(nphases)]);
    run_SLIP_high_gpops_new_2;
    info(cnt).nphases = nphases;
    info(cnt).time = trialtime;
    info(cnt).cost = output.result.objective;
    cnt = cnt + 1;
    
    save('Rebuttal_GPOPS_high', 'info');
end

save('Rebuttal_GPOPS_high', 'info');