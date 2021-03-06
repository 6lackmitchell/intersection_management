find(infeas==1)
find(pvios>0)
find(successes==0)
find(dlock>0)

close all;

% dyn_mode = 'double_integrator';
dyn_mode = 'dynamic_bicycle_rdrive_1u';

data = trial_data(4); % Deadlock for nominal case
code = data.code;
t = data.t;
dt = 0.01;
x = data.x;
u = data.u;
u0 = data.u0;
sols = data.sols;
violations = data.vios;    
barriers = data.barrier;    
alphas   = data.alpha;    

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
%     if jj == 3
    plot(tt,u(1:ii,jj,1),'LineWidth',lw)
    plot(tt,u0(1:ii,jj,1),'LineWidth',lw)
%     end
%     plot(tt,atan(uNom(1:ii,jj,1)),'LineWidth',lw)
end
legend('u_{11}','\mu_{11}','u_{21}','\mu_{21}','u_{31}','\mu_{31}','u_{41}','\mu_{41}')
hold off

figure(3);
title('Control Inputs Y')
hold on
for jj = 1:nAgents
%     if jj == 3
    plot(tt,u(1:ii,jj,2),'LineWidth',lw)
    plot(tt,u0(1:ii,jj,2),':','LineWidth',lw)
%     end
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

figure(4);
title('Density Barrier')
hold on
for jj = 1:nAgents-1
   plot(tt,barriers(1:ii,jj),'LineWidth',lw)
end
hold off

figure(5);
title('Alpha Param')
hold on
for jj = 1:nAgents-1
   plot(tt,alphas(1:ii,jj),'LineWidth',lw)
end
hold off
% ylim([-0.1 1])
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

filename = "something"
moviename = erase(filename,'.mat');
cinematographer(dt,x(1:(ii),:,:),dyn_mode,obstacles,moviename)

