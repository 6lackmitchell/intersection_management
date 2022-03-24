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

H   = zeros(ii,6,1);
h0  = zeros(ii,6,1);
Lfh = zeros(ii,6,1);
for tcbf = 1:length(ii)
    [H(tcbf,:),h0(tcbf,:),Lfh(tcbf,:)] = get_safety_values(squeeze(x(tcbf,:,:)));
end

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
colors = ['b','k','g','r'];
for jj = 1:nAgents
%     if jj == 1
    plot(tt,u(1:ii,jj,2),'LineWidth',lw,'Color',colors(jj))
    plot(tt,u0(1:ii,jj,2),':','LineWidth',lw,'Color',colors(jj))
%     end
%     plot(tt,uNom(1:ii,jj,2),':','LineWidth',lw)
end
legend('u_{12}','\mu_{12}','u_{22}','\mu_{22}','u_{32}','\mu_{32}','u_{42}','\mu_{42}')
hold off

figure(4);
title('Control Inputs Y')
hold on
colors = ['b','k','g','r'];
for jj = 1:nAgents
    plot(tt,(sols(1:ii,jj,8)-u(1:ii,4,2))/(jj+1),'LineWidth',lw,'Color',colors(jj))
end
legend('w_{14}','w_{24}','w_{34}','w_{44}')
hold off

figure(5);
title('Lfh')
hold on
colors = ['b','k','g','r','y','m'];
for jj = 1:6
    plot(tt,Lfh(1:ii,jj),'LineWidth',lw,'Color',colors(jj))
end
legend('12','13','14','23','24','34')
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


%% Safety Helper
function [Ht,H0,LFH] = get_safety_values(x)
Na    = 4;
tmax  = 5;

sw = 1.0;
Lr = 1;

Nc  = factorial(Na-1);
Ht  = zeros(Nc,1);
H0  = zeros(Nc,1);
LFH = zeros(Nc,1);
cc  = 1;
ss  = 1;

% Loop through every scheduled agent for PCCA
for aa = 1:Na
    
    nc  = Na-aa;
    hw  = zeros(nc,1);
    hw0 = zeros(nc,1);
    Lfhw = zeros(nc,1);
    dd  = 1;
    
    % Loop through all other agents for interagent completeness
    for ii = aa+1:Na
        
        xa = x(aa,:);
        xi = x(ii,:);
              
        % dx and dy
        dx  = xa(1) - xi(1);
        dy  = xa(2) - xi(2);
        
        % dvx and dvy
        vax = xa(4)*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));
        vay = xa(4)*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));
        vix = xi(4)*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));
        viy = xi(4)*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));
        dvx = vax - vix;
        dvy = vay - viy;
        
        % Solve for minimizer of h
        kh       = 1000.0; % This does not lead to safety violations
        eps      = 1e-3;
        tau_star = -(dx*dvx + dy*dvy)/(dvx^2 + dvy^2 + eps);
        Heavy1   = heavyside(tau_star,kh,0);
        Heavy2   = heavyside(tau_star,kh,tmax);
        tau      = tau_star*Heavy1 - (tau_star - tmax)*Heavy2;

        % accelerations -- controlled (con) and uncontrolled (unc)
        axa_unc = -xa(4)^2/Lr*tan(xa(5))*(sin(xa(3)) + cos(xa(3))*tan(xa(5)));% - betadot(aa)*xa(4)*sin(xa(3))*sec(xa(5))^2;
        axi_unc = -xi(4)^2/Lr*tan(xi(5))*(sin(xi(3)) + cos(xi(3))*tan(xi(5)));% - betadot(ii)*xi(4)*sin(xi(3))*sec(xi(5))^2;
        aya_unc =  xa(4)^2/Lr*tan(xa(5))*(cos(xa(3)) - sin(xa(3))*tan(xa(5)));% + betadot(aa)*xa(4)*cos(xa(3))*sec(xa(5))^2;
        ayi_unc =  xi(4)^2/Lr*tan(xi(5))*(cos(xi(3)) - sin(xi(3))*tan(xi(5)));% + betadot(ii)*xi(4)*cos(xi(3))*sec(xi(5))^2;

        % dax and day
        dax_unc = axa_unc - axi_unc;
        day_unc = aya_unc - ayi_unc;
        
        % taudot
        tau_star_dot_unc = -(dax_unc*(2*dvx*tau_star + dx) + day_unc*(2*dvy*tau_star + dy) + (dvx^2 + dvy^2)) / (dvx^2 + dvy^2 + eps);
        Heavy_dot1_unc   = dheavyside(tau_star,kh,0)*tau_star_dot_unc;
        Heavy_dot2_unc   = dheavyside(tau_star,kh,tmax)*tau_star_dot_unc;
        tau_dot_unc      = tau_star_dot_unc*(Heavy1 - Heavy2) + tau_star*(Heavy_dot1_unc - Heavy_dot2_unc);

        % h and hdot (= Lfh + Lgh*u)
        h0   = dx^2 + dy^2 - (2*sw)^2;
        h    = dx^2 + dy^2 + tau^2*(dvx^2 + dvy^2) + 2*tau*(dx*dvx + dy*dvy) - (2*sw)^2;
        Lfh  = 2*(dx*dvx + dy*dvy) + 2*tau*(dvx^2 + dvy^2 + dx*dax_unc + dy*day_unc) + 2*tau_dot_unc*(dx*dvx + dy*dvy + tau*(dvx^2 + dvy^2)) + 2*tau^2*(dvx*dax_unc + dvy*day_unc);        


%         % Robust-Virtual CBF
%         a1    = 0.1;
%         kh0   = 1;
%         H     = h   + a1*(tau-1)*h0^(1/kh0);

        % Robust-Virtual CBF
        a1    = 0.1;
        tbar  = 0.5;
        tbar  = 1.0;
        kh0   = 1;
        k2    = max([tau-tbar,eps]);
        H     = h   + a1*k2*h0;
    
        % Inequalities: Ax <= b
        hw(dd)          = H;
        hw0(dd)         = h0;
        Lfhw(dd)        = Lfh;

        dd = dd + 1;
        ss = ss + 1;

    end

    Ht(cc:cc+(nc-1))  = hw;
    H0(cc:cc+(nc-1))  = hw0;
    LFH(cc:cc+(nc-1)) = Lfhw;

    cc = cc + nc;
    
end

end

function [H] = heavyside(x,k,offset)
%HEAVYSIDE Summary of this function goes here
%   Detailed explanation goes here
H = 1/2 * (1 + tanh(k*(x - offset)));
end


