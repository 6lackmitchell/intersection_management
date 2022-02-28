clear; clc; 
% filename = strcat('datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/nominal_cbf/ff_cbf_4MonteCarlo_N1000.mat');
filename = strcat('datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/ff_cbf_4MonteCarlo_N1000_lookahead5_ffnorv.mat');
% filename = strcat('datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/ff_cbf_4MonteCarlo_N1000_lookahead5_GOAT_RVCBF.mat');

load(filename);

TTI     = Inf*ones(nTrials*nAgents,1);
vvios   = zeros(nTrials,1);
pvios   = zeros(nTrials,1);
infeas  = zeros(nTrials,1);
dlock   = zeros(nTrials,1);
endtime = zeros(nTrials,1);
successes = zeros(nTrials,1);
vio_mags  = zeros(nTrials,1);
for nn = 1:nTrials
    TTI((nn-1)*nAgents+1:(nn-1)*nAgents+nAgents) = trial_data(nn).TTI;
    infeas(nn)  = trial_data(nn).code == 0;
    endtime(nn) = trial_data(nn).t;
    dlock(nn)   = endtime(nn) == 20;
    successes(nn) = trial_data(nn).success;
    vvios(nn)   = sum(trial_data(nn).vios(:,:,1),'all') > 0;
    pvios(nn)   = sum(trial_data(nn).vios(:,:,2),'all') > 0;
    if pvios(nn) > 0
        vio_mags(nn) = min(trial_data(nn).vmags);
    end
end

success_rate  = sum(successes) / nTrials
successes_idx = find(successes == 1);
if isempty(successes_idx)
    average_time = 0
else
    average_time  = sum([trial_data(successes_idx).t]) / sum(successes)
end

sortedTTI  = sort(TTI);
finished   = sortedTTI(find(sortedTTI < 5.0));
unfinished = sortedTTI(find(sortedTTI >= 5.0));

fraction_finished   = length(finished) / (nAgents*nTrials);
fraction_unfinished = 1 - fraction_finished;

fraction_infeasible = sum(infeas) / nTrials;

fraction_complete   = 1 - fraction_infeasible - sum(pvios) / nTrials;
fraction_feasible   = 1 - fraction_infeasible
fraction_deadlock   = sum(dlock) / nTrials

% fraction_virt_vio   = sum(vvios) / nTrials
fraction_phys_vio   = sum(pvios) / nTrials
avg_phys_vio        = mean(vio_mags(find(vio_mags < 0)))

mean_all            = mean(finished,'all');
mean_endtime        = mean(endtime(find(infeas==1))); 