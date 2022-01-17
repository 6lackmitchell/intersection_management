data = trial_data(992);
% data = trial_data(994);
t = data.t;
dt = 0.01;
x = data.x;
u = data.u;



ii = fix(t / dt);
tt = linspace(dt,ii*dt,ii);

filename = strcat('datastore/',dyn_mode,'/',con_mode,'_',num2str(nAgents),'intersection_tests.mat');


figure(2);
title('Control Inputs X')
hold on
for jj = 1:nAgents
    plot(tt,u(1:ii,jj,1),'LineWidth',lw)
%     plot(tt,atan(uNom(1:ii,jj,1)),'LineWidth',lw)
end
legend('\omega_1','\omega_2','\omega_3','\omega_4','\omega_5','\omega_6')
hold off

figure(3);
title('Control Inputs Y')
hold on
for jj = 1:nAgents
    plot(tt,u(1:ii,jj,2),'LineWidth',lw)
%     plot(tt,uNom(1:ii,jj,2),':','LineWidth',lw)
end
legend('a_1','a_2','a_3','a_4','a_5','a_6')
hold off

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
cinematographer(dt,x(1:(ii),:,:),obstacles,moviename)

