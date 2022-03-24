clear;
% filename = 'C:\Users\DASC\Documents\git\intersection_management\datastore\intersection_crossing_straight\dynamic_bicycle_rdrive_1u\d_css\no_backup\input_constraints\no_pcca\nominal_cbf\no_priority\ff_cbf_4MonteCarlo_N1000_Nnon0_K10.mat'
filepath = 'C:\Users\DASC\Documents\git\intersection_management\datastore\intersection_crossing_turning\dynamic_bicycle_rdrive_1u\d_css\no_backup\input_constraints\no_pcca\rv_cbf\no_priority\';
filename = strcat(filepath,'ff_cbf_4MonteCarlo_N1000_Nnon0_K10.mat');
load(filename)

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
    dlock(nn)   = endtime(nn) == 20 || trial_data(nn).code == -2;
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
avg_phys_vio        = mean(vio_mags(find(vio_mags < 0)));
max_phy_vio         = max(abs(vio_mags(find(vio_mags < 0))))

mean_all            = mean(finished,'all');
mean_endtime        = mean(endtime(find(infeas==1))); 

%%
find(infeas==1)
find(pvios>0)
% find(successes==0)
find(dlock>0)
%%

clf;

% dyn_mode = 'double_integrator';
dyn_mode = 'dynamic_bicycle_rdrive_1u';

% data = trial_data(582); % Deadlock for nominal case
% data = trial_data(137); % Infeasible for ff case
data = trial_data(9); % Success for rv case
code = data.code;
t = data.t;
dt = 0.01;
x = data.x;
u = data.u;
u0 = data.u0;
sols = data.sols;
violations = data.vios;    

ii = fix(t / dt);
tt = linspace(dt,ii*dt,ii);

% filename = strcat('datastore/',dyn_mode,'/',con_mode,'_',num2str(nAgents),'intersection_tests.mat');

% for jj = 1:nAgents
%     v_vios = sum(violations(:,jj,1))
%     p_vios = sum(violations(:,jj,2))
% end

figure(2);
title('Control Inputs X')
hold on
for jj = 1:nAgents
    if jj == 1
    plot(tt,u(1:ii,jj,1),'LineWidth',lw)
    plot(tt,u0(1:ii,jj,1),'LineWidth',lw)
    end
%     plot(tt,atan(uNom(1:ii,jj,1)),'LineWidth',lw)
end
legend('u_{11}','\mu_{11}','u_{21}','\mu_{21}','u_{31}','\mu_{31}','u_{41}','\mu_{41}')
hold off

figure(3);
title('Control Inputs Y')
hold on
for jj = 1:nAgents
    if jj == 1
    plot(tt,u(1:ii,jj,2),'LineWidth',lw)
    plot(tt,u0(1:ii,jj,2),'LineWidth',lw)
    end
%     plot(tt,uNom(1:ii,jj,2),':','LineWidth',lw)
end
legend('u_{12}','\mu_{12}','u_{22}','\mu_{22}','u_{32}','\mu_{32}','u_{42}','\mu_{42}')
hold off

% figure(4);
% title('Slack Variables')
% hold on
% for jj = 1:nAgents
%     for ss = 1:factorial(nAgents-1)
%         plot(tt,sols(1:ii,jj,nAgents*nControls+ss),'LineWidth',lw)
%     end
% end
% legend('\delta_1','\delta_2','\delta_3','\delta_4','\delta_5','\delta_6')
% hold off

% figure(4);
% title('Violations')
% hold on
% for jj = 1:nAgents
%     vvios = sum(violations(:,jj,1))
%     pvios = sum(violations(:,jj,2))
% end
% legend('\delta_1','\delta_2','\delta_3','\delta_4','\delta_5','\delta_6')
% hold off

% figure(4);
% title('CBFs')
% hold on
% for jj = 1:nAgents
%     plot(tt,safety(1:ii,jj),'LineWidth',lw)
% end
% legend('h_1','h_2','h_3','h_4','h_5','h_6')
% hold off
% 
% figure(5);
% title('Gammas')
% hold on
% % plot(tt,gammas(1:ii,1,1),'LineWidth',lw)
% % plot(tt,gammas(1:ii,2,2),'LineWidth',lw)
% % plot(tt,gammas(1:ii,3,3),'LineWidth',lw)
% plot(tt,gammas(1:ii,4,1),'LineWidth',lw)
% plot(tt,gammas(1:ii,4,2),'LineWidth',lw)
% plot(tt,gammas(1:ii,4,3),'LineWidth',lw)
% % plot(tt,gammas(1:ii,4,4),'LineWidth',lw)
% % plot(tt,gammas(1:ii,jj,4),'LineWidth',lw)
% % for jj = 1:nAgents
% %     plot(tt,gammas(1:ii,jj,1),'LineWidth',lw)
% %     plot(tt,gammas(1:ii,jj,2),'LineWidth',lw)
% %     plot(tt,gammas(1:ii,jj,3),'LineWidth',lw)
% %     plot(tt,gammas(1:ii,jj,4),'LineWidth',lw)
% % end
% legend('\gamma_1','\gamma_2','\gamma_3')%,'\gamma_1','\gamma_1','\gamma_1')
% hold off
% 
% figure(6);
% title('aVals')
% hold on
% for jj = 1:nAgents
%     plot(tt,avalues(1:ii,jj),'LineWidth',lw)
% end
% legend('a_1','a_2','a_3','a_4')
% hold off

% figure(7);
% title('bVals')
% hold on
% for jj = 1:nAgents
%     plot(tt,bvalues(1:ii,jj),'LineWidth',lw)
% end
% legend('a_1','a_2','a_3','a_4')
% hold off

% Load road geometry
road_file = 'datastore/geometry/road_markings.mat';       
load(road_file)

moviename = erase(filename,'.mat');
cinematographer(dt,x(1:(ii),:,:),dyn_mode,obstacles,moviename)

