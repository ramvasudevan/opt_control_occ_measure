clear;

totaltime = [];
mincost = inf;
for nphases = 4 : 12
    run_SLIP_high_gpops_new_2;
    totaltime = [ totaltime ; trialtime ];
    if ~isempty( output.result.objective )
        if mincost > output.result.objective
            mincost = output.result.objective;
        end
    end
end

disp(['total time = ', num2str(sum(totaltime))]);
disp(['min cost = ', num2str(mincost)]);

save('Rebuttal_GPOPS_high');