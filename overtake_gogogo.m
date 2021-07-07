clc; close all;
global lane options

options = optimoptions('quadprog','Display','off');
set(0,'DefaultLegendAutoUpdate','off')
txt   = 'obtain_dV_dh';
fname = ['20200330_CDC_' txt '.mat'];

% Simulation Parameters
M        = 1650;        % Vehicle Mass (kg)
g        = 9.81;     % Accel due to gravity (m / sec^2)
tf       = 30;      % Final time (sec)
dt       = 0.001;    % Timestep (sec)
epsilon  = 0;        % Allowable error in Lyapunov Convergence
car      = 2.27 / 2; % Half Width of Car
tau      = 1.8;      % Time behind vehicle 1
lane     = 3;        % Distance in meters from center of one lane to next

% Recording Variables
nIter      = 1;
rev        = 1;
e_all      = zeros(rev,nIter,1);
q_rec      = zeros(nIter,tf/dt,12);
u_rec      = zeros(nIter,tf/dt,2);
delta1_rec = zeros(rev,nIter,tf/dt,1);
qbar_rec   = zeros(nIter,tf/dt,12);
hg_rec     = zeros(nIter,tf/dt,1);
dhg_rec    = zeros(nIter,tf/dt,1);
hs_rec     = zeros(nIter,tf/dt,3);
dhs_rec    = zeros(nIter,tf/dt,3,1);
Lf_rec     = zeros(nIter,tf/dt,3);
Lg_rec     = zeros(nIter,tf/dt,3,2);
phi_rec    = zeros(nIter,tf/dt,2);
T_rec      = zeros(rev,nIter,3);

% Initial Velocities of Vehicles (m/sec)
v10     = [18; 18; 18; 20; 20; 20; 16; 16; 16;];
v20     = [25; 25; 25; 20; 20; 20; 22; 22; 22;];
v10     = [16; 16.5; 17; 17.5; 18; 18.5; 19];
v10     = [17; 17; 17; 17; 17; 17; 17; 17; 17; 17];
% v10     = [17; 17; 17; 16.9; 16.9; 16.9; 17.1; 17.1; 17.1; 17.05];
v20     = [25; 25; 25; 25; 25; 25; 25; 25; 25; 25; 25];
          
% Input Disturbance Initial Conditions
max_dVdq = 0.08;
d1_max   = 0;1/ 2 * 0.638 / max_dVdq;
phi_inf  = d1_max * [0.1; 0.2; 0.3; 0.4; 0.5; 0.6; 0.7; 0.8; 0.9; 1.0];

% Initial Orientations (rad)
theta1 = 0;
theta2 = pi;
thetae = 0;

% Lyapunov Function Multiplier
K = 0.0001;
U = [1; 1; 1; 1; 1; 1; 1; 1; 1; 1;];

% Sim
for s = 1:rev
for i = 1:nIter
    
        % Minimum time to pass while in left lane (based on initial/final
        % desired x-coordinates
        T0 = 2*tau*v10(i) / (25 - v10(i)) + 2;

        % Parameters to vary for d1 trials 
        T          = [10, T0 + 2,      6;
                      10, T0 + 2,      6;
                      10, T0 + 2,      6;
                      10, T0 + 2,      6;
                      10, T0 + 2,      6;
                      10, T0 + 2,      6;
                      10, T0 + 2,      6;
                      10, T0 + 2,      6;
                      10, T0 + 2,      6;
                      10, T0 + 2,      6;
                      10, T0 + 2,      6;
                      10, T0 + 2,      6;];
%                         3,  T0 - 2,    2.5;
%                       4,  T0 - 1.5,    3;
%                       5,  T0 - 1,    3.5;
%                       6,  T0 - 0.5,    4;
%                       7,  T0 + 0,    4.5;
%                       8,  T0 + 0.5,    5;
%                       9,  T0 + 1,    5.5;
%                       10, T0 + 2,      6;
%                       11, T0 + 2.5,    6;
%                       12, T0 + 3,      6;];

        T_worst = compute_T_worst(T(s,:),d1_max*max_dVdq);
        disp(['T worst: ' num2str(T_worst)])

        t_oncoming = [2; 30; 4; 2; 30; 4; 2; 30; 2; 4];

        % Initial Position Conditions (m)
        x1 =  tau*v10(i);                                   
        y1 = lane / 2;
        x2 = -tau*v10(i) + (v10(i)+v20(i)) * t_oncoming(i); 
        y2 = lane * (2 - 1 / 2);
        xe = -tau*v10(i);                                   
        ye = y1;

        % Initial State Assignment
        q = [xe; ye; thetae; v10(i);
             x1; y1; theta1; v10(i);
             x2; y2; theta2; v20(i)];

        % Recording Variables
        ego_end_pos = zeros(4,2);
        other_end_pos = zeros(4,2);
        second_end_pos = zeros(4,2);

        % Convergence Params
        t_convergence       = tf/dt - 10;
        converged           = false;
        ego_end_pos(1,:)    = [q(1) q(2)];
        other_end_pos(1,:)  = [q(5) q(6)];
        second_end_pos(1,:) = [q(9) q(10)];

        % Desired State
        xd_rel1 = -1.5*tau*q(4) + 50;
        yd_rel1 = 0;
        vd      = v10(i);

        t  = 1;
        hg = 99;
        passing = false;

        % Ego Vehicle moves into Next Lane
        while t*dt < tf && ~converged

            % Do not allow advance to passing segment until safe wrt Car2
            safe_to_pass = q(9) + (q(12)*cos(q(11)) - q(4)*cos(q(3))) * T_worst > q(1) || q(1) > q(9);
            if safe_to_pass && ~passing
                yd_rel1 = lane;
                vd = 25;
                passing = true;
                disp(['Changing Lanes! t = ' num2str(t*dt)])
                st = t*dt;
            end

            qd = qd_func(q,xd_rel1,yd_rel1,vd);

            [q,u,d1,hg,dhg,hs,dhs,umax,phi,sf_violated] = move(q,qd,lane,tau,phi_inf(i),K,U(i),T(1),dt,M);

            q_rec(i,t,1:length(q))     = q;
            u_rec(i,t,1:2)             = u;
            delta1_rec(s,i,t)          = 2*d1;
            qbar_rec(i,t,1:length(q))  = qd-q;
            hg_rec(i,t)                = hg;
            dhg_rec(i,t,:)             = dhg;
            hs_rec(i,t,:)              = hs;
            dhs_rec(i,t,:,:)             = dhs;
            phi_rec(i,t,:)             = phi;

            ft = t;

            converged = (hg <= epsilon) && passing;
            if converged
                t_convergence = ft;
            end

            t  = t + 1;

            if sf_violated
                break
            end

        end

        et = t*dt;
        e_all(i) = t - 1;

        if sf_violated
            continue
        end

        update_progress(q,qd,t,dt);
        disp(['Time of Segment 1: ' num2str(et-st) ' sec. PT = ' num2str(T(1)) ' sec'])

        T_rec(s,i,1) = et-st;
        ego_end_pos(2,:)   = [q(1) q(2)];
        other_end_pos(2,:) = [q(5) q(6)];
        second_end_pos(2,:) = [q(9) q(10)];
        converged          = false;

        % Desired State
        xd_rel1 = 1.5*tau*q(4) + 50;
        yd_rel1 = lane;
        vd      = 25;

        st = t*dt;

        % Ego Vehicle Overtakes Vehicle
        while t*dt < tf && ~converged

            qd = qd_func(q,xd_rel1,yd_rel1,vd);

            [q,u,d1,hg,dhg,hs,dhs,umax,phi,sf_violated] = move(q,qd,lane,tau,phi_inf(i),K,U(i),T(2),dt,M);

            q_rec(i,t,1:length(q))     = q;
            u_rec(i,t,1:2)             = u;
            delta1_rec(s,i,t)          = 2*d1;
            qbar_rec(i,t,1:length(q))  = qd-q;
            hg_rec(i,t)                = hg;
            dhg_rec(i,t,:)             = dhg;
            hs_rec(i,t,:)              = hs;
            dhs_rec(i,t,:,:)             = dhs;
            phi_rec(i,t,:)             = phi;

            ft = t;

            converged = (hg <= epsilon);
            if converged
                t_convergence = ft;
            end

            t  = t + 1;

            if sf_violated
                break
            end

        end

        et = t*dt;
        e_all(i) = t - 1;

        if sf_violated
            continue
        end

        update_progress(q,qd,t,dt);
        disp(['Time of Segment 2: ' num2str(et-st) ' sec. PT = ' num2str(T(2)) ' sec'])
        T_rec(s,i,2) = et-st;
        
        ego_end_pos(3,:)    = [q(1) q(2)];
        other_end_pos(3,:)  = [q(5) q(6)];
        second_end_pos(3,:) = [q(9) q(10)];
        converged           = false;

        % Desired State
        xd_rel  = T*q(4);
        xd_rel1 = 1.5*tau*q(4) + 50;
        yd_rel1 = 0;
        vd      = 25;

        st = t*dt;

        % Ego Vehicle Returns to Primary Lane
        while t*dt < tf && ~converged

            qd = qd_func(q,xd_rel1,yd_rel1,vd);

            [q,u,d1,hg,dhg,hs,dhs,umax,phi,sf_violated] = move(q,qd,lane,tau,phi_inf(i),K,U(i),T(3),dt,M);

            q_rec(i,t,1:length(q))     = q;
            u_rec(i,t,1:2)             = u;
            delta1_rec(s,i,t)          = 2*d1;
            qbar_rec(i,t,1:length(q))  = qd-q;
            hg_rec(i,t)                = hg;
            dhg_rec(i,t,:)             = dhg;
            hs_rec(i,t,:)              = hs;
            dhs_rec(i,t,:,:)           = dhs;
            phi_rec(i,t,:)             = phi;

            ft = t;

            converged = (hg <= epsilon);
            if converged
                t_convergence = ft;
            end

            t  = t + 1;

            if sf_violated
                break
            end

        end

        et = t*dt;
        e_all(i) = t - 1;

        if sf_violated
            continue
        end

        update_progress(q,qd,t,dt);
        disp(['Time of Segment 3: ' num2str(et-st) ' sec. PT = ' num2str(T(3)) ' sec'])
        T_rec(s,i,3) = et-st;
            
        ego_end_pos(4,:)   = [q(1) q(2)];
        other_end_pos(4,:) = [q(5) q(6)];
        second_end_pos(4,:) = [q(9) q(10)];

        m = max(delta1_rec(s,i,:));
        disp(['Max Delta1: ' num2str(m)])
end
end

beep
pause(0.5)
beep
pause(0.1)
beep

m = max(delta1_rec);
disp(['Max Delta1: ' num2str(m)])

save(fname);

% Plots

plot_t = 1:tf/dt;
s      = 1;
e      = ft;
i = 3;
%% OG Display All Trajectories
colors = ['b' 'r' 'y' 'c' 'm' 'k' 'g' 'b' 'r' 'c'];
figure(1);
e0 = e_all - 1;
e  = max(e0);
p = zeros(nIter,1);
l0 = zeros(length(q_rec(1,s:e,1)),1);
l2 =  3*ones(length(q_rec(1,s:e,1)),1);
l3 =  6*ones(length(q_rec(1,s:e,1)),1);
hold on
for i = 1:nIter
    p(i) = plot(q_rec(i,s:e0(i),1),q_rec(i,s:e0(i),2),'DisplayName',['v_o = ' num2str(v10(i)) 'm/s, t_p = ' num2str(t_oncoming(i)) 'sec'],'LineWidth',4,'Color',colors(i));
end
plot(q_rec(i,s:e,1),l0,'k','LineWidth',5)
p2 = plot(q_rec(i,s:e,1),l2,'--k','LineWidth',3,'DisplayName','Lane Divider');
plot(q_rec(i,s:e,1),l3,'k','LineWidth',5);
hold off
set(gca,'fontsize',40)
xlabel('Longitudinal Distance, X (m)','fontsize',60);
ylabel('Transverse Distance, Y (m)','fontsize',60);
legend([p; p2])
ylim([-0.5 13])

%%
figure(20);
subplot(2,1,1)
l = max(e_all(1:nIter)-1);
p = zeros(2*nIter,1);
hold on
for i = 1:nIter
    p(i) = plot(plot_t(s:e_all(i)-1)*dt,u_rec(i,s:e_all(i)-1,1),'LineWidth',3,'Color',colors(i),'DisplayName','\omega (t)');
end
p10 = plot(plot_t(s:l)*dt,wmax*ones(length(plot_t(s:l)),1),'k--','DisplayName','\omega _m, \omega _M');
plot(plot_t(s:l)*dt,wmax*ones(length(plot_t(s:l)),1)*-1,'k--')
hold off
legend([p(1); p10],'fontsize',40)
set(gca,'fontsize',40)
ylabel('\omega (t)','fontsize',60)
% xlim([0 65])

subplot(2,1,2)
hold on
for i = 1:nIter
    p(nIter + i) = plot(plot_t(s:e_all(i)-1)*dt,u_rec(i,s:e_all(i)-1,2)/(M*g),'LineWidth',3,'Color',colors(i),'DisplayName','a(t)');
end
p11 = plot(plot_t(s:l)*dt,0.25*M*ones(length(plot_t(s:l)),1),'k--','DisplayName','a_m, a_M');
plot(plot_t(s:l)*dt,0.25*M*ones(length(plot_t(s:l)),1)*-1,'k--')
hold off
set(gca,'fontsize',40)
% legend([p(11); p11],'fontsize',40)

xlabel('Time (sec)','fontsize',60)
ylabel('a(t) / Mg','fontsize',60);
ylim([-0.4 0.4])
% xlim([0 65])

figure(6);
hold on
for i = 1:nIter
    plot(plot_t(s:e_all(i)-1)*dt,q_rec(i,s:e_all(i)-1,4),'DisplayName',num2str(i),'LineWidth',3)%,plot_t(s:e_all(i)-1)*dt,q_rec(i,s:e_all(i)-1,8),plot_t(s:e_all(i)-1)*dt,q_rec(i,s:e_all(i)-1,12))
end
hold off
title('Vehicle Velocity');
xlabel('Time (sec)');
ylabel('Velocity (m/s)');
legend('show')
% legend({'Controlled Vehicle','Parallel Vehicle','Antiparallel Velocity'});

% 
% %%
% figure(4);
% for i = 1:nIter
%     hold on
%     l0 = zeros(length(q_rec(i,s:e,1)),1);
%     l1 = -3*ones(length(q_rec(i,s:e,1)),1);
%     l2 =  3*ones(length(q_rec(i,s:e,1)),1);
%     l3 =  6*ones(length(q_rec(i,s:e,1)),1);
%     l4 =  9*ones(length(q_rec(i,s:e,1)),1);
%     plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['Iteration ' num2str(i)],'LineWidth',4)
% %     plot(q_rec(i,s:e,1),q_rec(i,s:e,2),'DisplayName',['Iteration ' num2str(i)],'LineWidth',4)
% %     plot(...
% %     ...%      q_rec(nIter,s:e,1),q_rec(nIter,s:e,2) + car,...
% %     ...%      q_rec(nIter,s:e,1),q_rec(nIter,s:e,2) - car',...
% %          q_rec(i,s:e,5),q_rec(i,s:e,6),...
% %          q_rec(i,s:e,9),q_rec(i,s:e,10),...
%          plot(q_rec(i,s:e,1),l0,'k--',...
%          q_rec(i,s:e,1),l1,'k--',...
%          q_rec(i,s:e,1),l2,'k--',...
%          q_rec(i,s:e,1),l3,'k--',...
%          q_rec(i,s:e,1),l4,'k--')%,...
% %          ego_end_pos(1,1),ego_end_pos(1,2),'bx',...
% %          other_end_pos(1,1),other_end_pos(1,2),'bo',...
% %          [ego_end_pos(1,1),other_end_pos(1,1)],[ego_end_pos(1,2),other_end_pos(1,2)],':',... 
% %          ego_end_pos(2,1),ego_end_pos(2,2),'rx',...
% %          other_end_pos(2,1),other_end_pos(2,2),'ro',...
% %          [ego_end_pos(2,1),other_end_pos(2,1)],[ego_end_pos(2,2),other_end_pos(2,2)],':',...
% %          ego_end_pos(2,1),ego_end_pos(2,2),'rx',...
% %          second_end_pos(2,1),second_end_pos(2,2),'ro',...
% %          [ego_end_pos(2,1),second_end_pos(2,1)],[ego_end_pos(2,2),second_end_pos(2,2)],':',...
% %          ego_end_pos(3,1),ego_end_pos(3,2),'gx',...
% %          other_end_pos(3,1),other_end_pos(3,2),'go',...
% %          [ego_end_pos(3,1),other_end_pos(3,1)],[ego_end_pos(3,2),other_end_pos(3,2)],':',...
% %          ego_end_pos(3,1),ego_end_pos(3,2),'rx',...
% %          second_end_pos(3,1),second_end_pos(3,2),'ro',...
% %          [ego_end_pos(3,1),second_end_pos(3,1)],[ego_end_pos(3,2),second_end_pos(3,2)],':',...
% %          ego_end_pos(4,1),ego_end_pos(4,2),'cx',...
% %          other_end_pos(4,1),other_end_pos(4,2),'co',...
% %          [ego_end_pos(4,1),other_end_pos(4,1)],[ego_end_pos(4,2),other_end_pos(4,2)],':',...
% %          ego_end_pos(5,1),ego_end_pos(5,2),'kx',...
% %          other_end_pos(5,1),other_end_pos(5,2),'ko',...
% %          [ego_end_pos(5,1),other_end_pos(5,1)],[ego_end_pos(5,2),other_end_pos(5,2)],':',...
% %          ego_end_pos(6,1),ego_end_pos(6,2),'mx',...
% %          other_end_pos(6,1),other_end_pos(6,2),'mo',...
% %          [ego_end_pos(6,1),other_end_pos(6,1)],[ego_end_pos(6,2),other_end_pos(6,2)],':')
% end
% hold off
% title('Vehicle Trajectory');
% xlabel('Longitudinal Distance (X - m)');
% ylabel('Transverse Distance (Y - m)');
% legend('show')% legend({'Controlled Vehicle','Controlled Vehicle Edge1','Controlled Vehicle Edge2','Parallel Vehicle'})%,'Antiparallel Velocity'});
% xlim([xe q(1)])
% ylim([-3.5 9.5])
% 
% %% Oncoming Vehicle!
% figure(4);
% p = zeros(nIter+3,1);
% colors = ['b' 'r' 'y' 'c' 'm' 'k' 'g' 'b' 'r' 'c'];
% i = 2;
% l0 = zeros(length(q_rec(i,s:e,1)),1);
% l2 = 3*ones(length(q_rec(i,s:e,1)),1);
% l3 = 6*ones(length(q_rec(i,s:e,1)),1);
% 
% s1 = subplot(3,1,1);
% hold on
% yyaxis right
% p(11) = plot(q_rec(2,s:e,1),l2,'Color','#000000','LineStyle','--','LineWidth',3,'DisplayName','Lane Line');
% % i = 1;
% % p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
% i = 4;
% p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
% i = 7;
% p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
% i = 9;
% p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
% i = 3;
% plot(q_rec(2,s:e,1),l0,'k','LineWidth',4)
% plot(q_rec(2,s:e,1),l3,'k','LineWidth',4)
% hold off
% legend([p(4),p(7),p(9)],'fontsize',40)
% ax = gca;
% ax.YAxis(1).FontSize = 0.1;
% ax.YAxis(2).FontSize = 40;
% ax.YAxis(2).Color = 'k';
% ylim([-0.5 6.5])
% txt = ['t_p = ' num2str(t_oncoming(1)) ' sec'];
% text(-10,5,txt,'fontsize',40)
% xlim([xe q_rec(2,e_all(2)-1) + 275])
% 
% s2 = subplot(3,1,2);
% hold on
% yyaxis right
% i = 3;
% p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
% i = 6;
% p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
% i = 10;
% p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
% p(13) = plot(q_rec(2,s:e,1),l2,'Color','#000000','LineStyle','--','LineWidth',3,'DisplayName','Lane Line');
% plot(q_rec(2,s:e,1),l0,'k','LineWidth',4)
% plot(q_rec(2,s:e,1),l3,'k','LineWidth',4)
% hold off
% legend([p(3),p(6),p(10)],'fontsize',40)
% xlim([xe q_rec(2,e_all(2)-1) + 275])
% ax = gca;
% ax.YAxis(1).FontSize = 0.1;
% ax.YAxis(2).FontSize = 40;
% ax.YAxis(2).Color = 'k';
% ylim([-0.5 6.5])
% yyaxis left
% ylabel('Transverse Distance, Y (m)','fontsize',60,'Color','k');
% txt = ['t_p = ' num2str(t_oncoming(3)) ' sec'];
% text(-10,5,txt,'fontsize',40)
% 
% 
% s3 = subplot(3,1,3);
% hold on
% yyaxis right
% i = 2;
% p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
% i = 5;
% p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
% i = 8;
% p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
% i = 3;
% p(12) = plot(q_rec(2,s:e,1),l2,'Color','#000000','LineStyle','--','LineWidth',3,'DisplayName','Lane Line');
% plot(q_rec(2,s:e,1),l0,'k','LineWidth',4)
% plot(q_rec(2,s:e,1),l3,'k','LineWidth',4)
% hold off
% legend([p(2),p(5),p(8)],'fontsize',40)
% ax = gca;
% ax.YAxis(1).FontSize = 0.1;
% ax.YAxis(2).FontSize = 40;
% ax.YAxis(2).Color = 'k';
% ylim([-0.5 6.5])
% xlim([xe q_rec(2,e_all(2)-1) + 275])
% xlabel('Longitudinal Distance, X (m)','fontsize',60);
% txt = ['t_p = ' num2str(t_oncoming(2)) ' sec'];
% text(-10,5,txt,'fontsize',40)
% 
% pos1 = get(s1,'Position');
% pos1(4) = pos1(4) + 0.07;
% set(s1,'Position',pos1);
% pos2 = get(s2,'Position');
% pos2(4) = pos2(4) + 0.01;
% set(s2,'Position',pos2);
% pos3 = get(s3,'Position');
% pos4(4) = pos3(4) + 0.03;
% set(s3,'Position',pos3);
% 
% %%
% %% Full Slate for one Test
% figure(4);
% wmax = pi/18;
% i = 5;
% colors = ['b' 'r' 'y' 'c' 'm' 'k' 'g' 'b' 'r' 'c'];
% subplot(3,1,1)
% hold on
% l0 = zeros(length(q_rec(i,s:e,1)),1);
% l2 = 3*ones(length(q_rec(i,s:e,1)),1);
% l3 = 6*ones(length(q_rec(i,s:e,1)),1);
% p1 = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['v_o = ' num2str(v10(i)) 'm/s , t_p = ' num2str(round(t_oncoming(i),1)) ' sec'],'LineWidth',3,'Color',colors(i));
% p2 = plot(q_rec(i,s:e,1),l2,'Color','#000000','LineStyle','--','LineWidth',3,'DisplayName','Lane Line');
% plot(q_rec(i,s:e,1),l0,'k','LineWidth',5)
% plot(q_rec(i,s:e,1),l3,'k','LineWidth',5)
% hold off
% set(gca,'FontSize',30)
% xlabel('Longitudinal Distance, X (m)','fontsize',50);
% ylabel('Transverse Distance, Y (m)','fontsize',50);
% % legend({p1, p2},'fontsize',30)
% xlim([xe q_rec(i,e_all(i)-1,1)])
% ylim([-0.5 9.5])
% 
% subplot(3,1,2)
% plot(plot_t(s:e)*dt,u_rec(nIter,s:e,1)./wmax,...
%      plot_t(s:e)*dt,u_rec(nIter,s:e,2)./umax,...
%      plot_t(s:e)*dt,ones(length(plot_t(s:e)),1),'k--',...
%      plot_t(s:e)*dt,ones(length(plot_t(s:e)),1)*-1,'k--')
% title('Control Input');
% ylabel('Control Input (Fraction of Mg)');
% ylim([-1.2 1.2])
% legend({'Angular Velocity Control (rad/s)','Longitudinal Acceleration (m/s^2)'})
% 
% subplot(3,1,3)
% 
% %%
% figure(5);
% plot(plot_t(s:e)*dt,q_rec(nIter,s:e,3),plot_t(s:e)*dt,q_rec(nIter,s:e,7),plot_t(s:e)*dt,q_rec(nIter,s:e,11))
% title('Vehicle Orientation');
% xlabel('Time (sec)');
% ylabel('Heading Angle (rad)');
% legend({'Controlled Vehicle','Parallel Vehicle','Antiparallel Velocity'});
% 
% %
% figure(6);
% hold on
% for i = 1:nIter
%     plot(plot_t(s:e_all(i)-1)*dt,q_rec(i,s:e_all(i)-1,4),'DisplayName',num2str(i),'LineWidth',3)%,plot_t(s:e_all(i)-1)*dt,q_rec(i,s:e_all(i)-1,8),plot_t(s:e_all(i)-1)*dt,q_rec(i,s:e_all(i)-1,12))
% end
% hold off
% title('Vehicle Velocity');
% xlabel('Time (sec)');
% ylabel('Velocity (m/s)');
% legend('show')
% % legend({'Controlled Vehicle','Parallel Vehicle','Antiparallel Velocity'});
% 
% %
% wmax = 2 * pi / 36;
% figure(7);
% subplot(2,1,1)
% plot(plot_t(s:e)*dt,u_rec(nIter,s:e,1)./wmax,...
%      plot_t(s:e)*dt,u_rec(nIter,s:e,2)./umax,...
%      plot_t(s:e)*dt,ones(length(plot_t(s:e)),1),'k--',...
%      plot_t(s:e)*dt,ones(length(plot_t(s:e)),1)*-1,'k--')
% title('Control Input');
% ylabel('Control Input (Fraction of Mg)');
% ylim([-1.2 1.2])
% legend({'\omega (rad/s)','a (m/s^2)'})
% subplot(2,1,2)
% plot(plot_t(s:e)*dt,phi_rec(nIter,s:e,1)./wmax,...
%      plot_t(s:e)*dt,phi_rec(nIter,s:e,2)./umax,...
%      plot_t(s:e)*dt,ones(length(plot_t(s:e)),1),'k--',...
%      plot_t(s:e)*dt,ones(length(plot_t(s:e)),1)*-1,'k--')
% xlabel('Time (sec)');
% ylabel('Disturbance (Fraction Maximum Control)');
% ylim([-1.2 1.2])
% legend({'phi_w','phi_u'})
% 
% %%
% figure(8);
% hold on
% colors = ['b' 'r' 'y' 'c' 'm' 'k' 'g' 'b' 'r' 'c'];
% 
% for i = 1:nIter
%     c = colors(i);
%     
% %     p      = zeros(3,length(s:e_all(i)-1));
% %     p(1,:) = hg_rec(i,s:e_all(i)-1);
% %     p(2,:) = hs_rec(i,s:e_all(i)-1,1);
% %     p(3,:) = hs_rec(i,s:e_all(i)-1,3);
% %     pt     = [plot_t(s:e_all(i)-1)*dt; plot_t(s:e_all(i)-1)*dt; plot_t(s:e_all(i)-1)*dt];
% %     
% %     size(pt)
% %     size(p)
% %     
% %     plot(pt,p,'LineWidth',3,'Color',c,'DisplayName',['CLF, Road CBF, Lead Car CBF: ' num2str(i)])
% 
% %     plot(plot_t(s:e)*dt,hg_rec(nIter,s:e)/max(abs(hg_rec(nIter,s:e))),plot_t(s:e)*dt,hs_rec(nIter,s:e,1)/max(abs(hs_rec(nIter,s:e,1))),'--',plot_t(s:e)*dt,hs_rec(nIter,s:e,2)/max(abs(hs_rec(nIter,s:e,2))),'--',plot_t(s:e)*dt,hs_rec(nIter,s:e,3)/max(abs(hs_rec(nIter,s:e,3))),'--')
%     p1 = plot(plot_t(s:e_all(i)-1)*dt,100*hg_rec(i,s:e_all(i)-1),'LineWidth',3,'Color',c);
%     p2 = plot(plot_t(s:e_all(i)-1)*dt,hs_rec(i,s:e_all(i)-1,1),'--','LineWidth',3,'Color',c);
%     p3 = plot(plot_t(s:e_all(i)-1)*dt,hs_rec(i,s:e_all(i)-1,3),':','LineWidth',3,'Color',c);
% end
% hold off
% % legend('show','fontsize',40)
% % title('CLF / CBF Evolution');
% set(gca,'FontSize',40)
% xlabel('Time (sec)','fontsize',60);
% ylabel('CLF / CBF Value','fontsize',60);
% % xlim([s e_all(9)*dt+2])
% ylim([-3 1])
% legend({'100 $\cdotp$ CLF','CBF: Road','CBF: Lead Car'},'Interpreter','latex');
% 
% %%
% figure(8);
% hold on
% % plot(plot_t(s:e)*dt,hg_rec(nIter,s:e)/max(abs(hg_rec(nIter,s:e))),plot_t(s:e)*dt,hs_rec(nIter,s:e,1)/max(abs(hs_rec(nIter,s:e,1))),'--',plot_t(s:e)*dt,hs_rec(nIter,s:e,2)/max(abs(hs_rec(nIter,s:e,2))),'--',plot_t(s:e)*dt,hs_rec(nIter,s:e,3)/max(abs(hs_rec(nIter,s:e,3))),'--')
% plot(plot_t(s:e_all(i)-1)*dt,hg_rec(i,s:e_all(i)-1),'DisplayName',['Goal ' num2str(i)])
% plot(plot_t(s:e_all(i)-1)*dt,hs_rec(i,s:e_all(i)-1,1),'--','DisplayName',['Road ' num2str(i)])
% plot(plot_t(s:e_all(i)-1)*dt,hs_rec(i,s:e_all(i)-1,2),'--')
% plot(plot_t(s:e_all(i)-1)*dt,hs_rec(i,s:e_all(i)-1,3),'--','DisplayName',['Car ' num2str(i)])
% hold off
% legend('show')
% title('CLF / CBF Evolution');
% xlabel('Time (sec)');
% ylabel('Function Value (Normalized across Segments)');
% ylim([-1 1])
% % legend({'Goal CLF','Safety CBF - Road','Safety CBF - Speed','Safety CBF - Vehicle1','Safety - Vehicle2'});
% 
% %
% figure(10);
% 
% hold on
% for i = 1:nIter
%     l0 = zeros(length(q_rec(i,s:e,1)),1);
%     l1 = -3*ones(length(q_rec(i,s:e,1)),1);
%     l2 =  3*ones(length(q_rec(i,s:e,1)),1);
%     l3 =  6*ones(length(q_rec(i,s:e,1)),1);
%     l4 =  9*ones(length(q_rec(i,s:e,1)),1);
%     plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['Iteration ' num2str(i)],'LineWidth',4)
% end
% 
%%
TTT = sum(T');
nU = rev;
max_deltas = zeros(nU,nIter);
c3_1 = zeros(nIter,1);
c3_2 = zeros(nIter,1);
c3_3 = c3_2;
for j = 1:nIter
    for i = 1:nU
        max_deltas(i,j) = max(delta1_rec(j,i,:));
    end
    c3_1(j) = pi*5/T(j,1);
    c3_2(j) = pi*5/T(j,2);
    c3_3(j) = pi*5/T(j,3);
end

figure
xlabel('Fixed Time, T, of Full Overtake Maneuver (sec)','fontsize',40)
xlim([12 32])
hold on
yyaxis right
plot(TTT,c3_1,'DisplayName','T_1 (Subproblem 1)','LineWidth',3)
plot(TTT,c3_2,'DisplayName','T_2 (Subproblem 2)','LineWidth',3)
plot(TTT,c3_3,'DisplayName','T_3 (Subproblem 3)','LineWidth',3)
ylim([0 5])
set(gca,'fontsize',40)
h = 'c_3 = 2$\sqrt{\alpha _1 \alpha _2}$ (sec^{-1}';
% set(h,'Interpreter','latex') 
ylim([0.5 7])
ylabel('$2\sqrt{c_1c_2} = \frac{\mu\pi}{T_i}$','Interpreter','latex','fontsize',50);


yyaxis left
for i = [4 6 7]%1:nIter%[1,3,10]
    plot(TTT,max_deltas(i,:),'DisplayName',['u = [' num2str(round(U(i)*pi/18,2)) '; ' num2str(round(U(i)*0.25*9.81,2)) ']'],'LineWidth',3)
end
set(gca,'fontsize',40)
ylabel('$c_3 = 2\max(\delta_1)$','Interpreter','latex','fontsize',50)
ylim([0.5 7])

h = legend('show','fontsize',40,'Box','on');
hold off
% %%
% figure(32);
% s1 = subplot(3,1,1);
% pos1 = get(s1,'Position');
% hold on
% nIter = 10;
% p1 = plot(phi_inf,ones(nIter,1)*T(1),'DisplayName','Fixed-Time Window: Segment 1','LineWidth',3,'LineStyle','--');
% for i = 1:10
%     if i == 1
%         p2 = plot(phi_inf,T_rec(i,:,1),'DisplayName','Actual Convergence Time: Segment 1','LineWidth',3);
%     else
%         plot(phi_inf,T_rec(i,:,1),'DisplayName','Actual Convergence Time: Segment 1','LineWidth',3);
%     end
%     p3 = plot(phi_inf,ones(10,1)*T_reca(1,3,1),'DisplayName','Unperturbed Convergence Time: Segment 1','LineWidth',3,'LineStyle',':');
% end
% plot([phi_inf(nIter); phi_inf(nIter)],[T_rec(10,nIter,1); T(1)],'X','DisplayName','max(\delta _1)','LineWidth',3,'MarkerSize',20)
% hold off
% set(gca,'fontsize',0.1)
% legend([p1; p3; p2],'fontsize',40)
% ax = gca;
% ax.YAxis(1).FontSize = 40;
% ax.YAxis(1).Color = 'k';
% 
% s2 = subplot(3,1,2);
% pos2 = get(s2,'Position');
% hold on
% p1 = plot(phi_inf,ones(nIter,1)*T(2),'DisplayName','Fixed-Time Window: Segment 2','LineWidth',3,'LineStyle','--');
% for i = 1:10
%     if i == 1
%         p2 = plot(phi_inf,T_rec(i,:,2),'DisplayName','Actual Convergence Time: Segment 2','LineWidth',3);
%     else
%         plot(phi_inf,T_rec(i,:,2),'DisplayName','Actual Convergence Time: Segment 2','LineWidth',3);
%     end
%     p3 = plot(phi_inf,ones(10,1)*T_reca(1,3,2),'DisplayName','Unperturbed Convergence Time: Segment 2','LineWidth',3,'LineStyle',':');
% end
% plot([phi_inf(nIter); phi_inf(nIter)],[T_rec(10,nIter,2); T(2)],'X','DisplayName','max(\delta _1)','LineWidth',3,'MarkerSize',20)
% hold off
% legend([p1; p3; p2],'fontsize',40)
% ax = gca;
% ax.YAxis(1).FontSize = 40;
% ax.XAxis(1).FontSize = 0.01;
% ax.YAxis(1).Color = 'k';
% yl = ylabel('Time (sec)','fontsize',60,'Color','k');
% yl.Position(1) = yl.Position(1) + .1;
% 
% s3 = subplot(3,1,3);
% pos3 = get(s3,'Position');
% hold on
% p1 = plot(phi_inf,ones(nIter,1)*T(3),'DisplayName','Fixed-Time Window: Segment 3','LineWidth',3,'LineStyle','--');
% for i = 1:10
%     if i == 1
%         p2 = plot(phi_inf,T_rec(i,:,3),'DisplayName','Actual Convergence Time: Segment 3','LineWidth',3);
%     else
%         plot(phi_inf,T_rec(i,:,3),'DisplayName','Actual Convergence Time: Segment 3','LineWidth',3);
%     end
%     p3 = plot(phi_inf,ones(10,1)*T_reca(1,3,3),'DisplayName','Unperturbed Convergence Time: Segment 3','LineWidth',3,'LineStyle',':');
% end
% plot([phi_inf(nIter); phi_inf(nIter)],[T_rec(10,nIter,3); T(3)],'X','DisplayName','max(\delta _1)','LineWidth',3,'MarkerSize',20)
% hold off
% legend([p1; p3; p2],'fontsize',40)
% ax = gca;
% ax.YAxis(1).FontSize = 40;
% ax.YAxis(1).Color = 'k';
% ax.XAxis(1).FontSize = 40;
% % ylim([T(3)-0.45, T(3)+0.02])
% xlabel('Bound on Disturbance, \phi','fontsize',60)
% 
% pos1 = get(s1,'Position');
% pos1(4) = pos1(4) + 0.03;
% set(s1,'Position',pos1);
% pos2 = get(s2,'Position');
% pos2(4) = pos2(4) + 0.05;
% set(s2,'Position',pos2);
% pos3 = get(s3,'Position');
% pos3(4) = pos3(4) + 0.01;
% pos3(2) = pos3(2) + 0.03;
% set(s3,'Position',pos3);

%%

% Function Definitions
% New Desired X Calculation
function d = xd_func(x,v,T)
    d = x - T*v;
end

function ret = qd_func(q,xd_rel1,yd_rel1,vd)
    global lane
    xd = q(5) + xd_rel1;
    yd = q(6) + yd_rel1;
    thetad = atan2(yd - q(2),xd - q(1));
    y = q(2);
    
    ret = [    xd;
               yd;
           thetad;
               vd;
               xd;
               yd;
           thetad;
               vd;
               xd;
               yd;
           thetad;
               vd;
          ];
      
end

function ret = qbar_func(q,qd)
    global xnorm ynorm bnorm vnorm
    ret =  [q(1) - qd(1);
            q(2) - qd(2);
            atan2(q(4) * sin(q(3)) - qd(4) * sin(qd(3)),q(4) * cos(q(3)) - qd(4) * cos(qd(3))) - qd(3);
            sqrt((q(4) * cos(q(3)) - qd(4) * cos(qd(3))) ^ 2 + (q(4) * sin(q(3)) - qd(4) * sin(qd(3))) ^ 2);
            q(5) - qd(5);
            q(6) - qd(6);
            atan2(q(8) * sin(q(7)) - qd(8) * sin(qd(7)),q(8) * cos(q(7)) - qd(8) * cos(qd(7))) - qd(7);
            sqrt((q(8) * cos(q(7)) - qd(8) * cos(qd(7))) ^ 2 + (q(8) * sin(q(7)) - qd(8) * sin(qd(7))) ^ 2);
            q(9) - qd(9);
            q(10) - qd(10);
            atan2(q(12) * sin(q(11)) - qd(12) * sin(qd(11)),q(12) * cos(q(11)) - qd(12) * cos(qd(11))) - qd(11);
            sqrt((q(12) * cos(q(11)) - qd(12) * cos(qd(11))) ^ 2 + (q(12) * sin(q(11)) - qd(12) * sin(qd(11))) ^ 2)];
            
    ret = ret ./ [xnorm ynorm bnorm vnorm xnorm ynorm bnorm vnorm xnorm ynorm bnorm vnorm]';
end

function update_progress(q,qd,t,dt)
    if mod(t,10000) == 0
        disp(['t = ' num2str(t*dt) ' sec'])
        disp(q(2) - qd(2))
    end
end

% Move Function
function [new_q,new_u,delta1,hg,sum_dhg,hs,sum_dhs,u_max,phi,safety_violation] = move(q,qd,lane,tau,phi_scale,K,U,T,dt,M)
global options
% move: Solves CLF/CBF constrained QP for control input u

    % This function formulates the problem of an ego_vehicle 'overtaking'
    % another vehicle as a Control Lyapunov Function / Control Barrier Function
    % constrained Quadratic Programming problem and advances the system
    % dynamics forward by one timestep.

    % QP Objective Parameters - still being tuned
    p1 = 1000; % Relaxation penalty on performance term
    p2 = 1;   % Relaxation penalty on safety term
    p3 = 1;
    q1 = 200;  % Penalty on positive value of p1

    q_real = q;
    fq_real = [  q(4) * cos(q(3));
                 q(4) * sin(q(3));
                                0;
                                0;
                 q(8) * cos(q(7));
                 q(8) * sin(q(7));
                                0;
                                0;
               q(12) * cos(q(11));
               q(12) * sin(q(11));
                                0;
                                0];

    % Generate Dynamics Disturbance (Zero-Mean Gaussian) 
    psi_inf = phi_scale * [0; 0; 0; 0; 1; 1; 1; 1; 1; 1; 1; 1];
    psi = [min(max(psi_inf(1) / 3  * randn(1),-psi_inf(1)), psi_inf(1));
           min(max(psi_inf(2) / 3  * randn(1),-psi_inf(2)), psi_inf(2));
           min(max(psi_inf(3) / 3  * randn(1),-psi_inf(3)), psi_inf(3));
           min(max(psi_inf(4) / 3  * randn(1),-psi_inf(4)), psi_inf(4));
           min(max(psi_inf(5) / 3  * randn(1),-psi_inf(5)), psi_inf(5));
           min(max(psi_inf(6) / 3  * randn(1),-psi_inf(6)), psi_inf(6));
           min(max(psi_inf(7) / 3  * randn(1),-psi_inf(7)), psi_inf(7));
           min(max(psi_inf(8) / 3  * randn(1),-psi_inf(8)), psi_inf(8));
           min(max(psi_inf(9) / 3  * randn(1),-psi_inf(9)), psi_inf(9));
           min(max(psi_inf(10) / 3 * randn(1),-psi_inf(10)),psi_inf(10));
           min(max(psi_inf(11) / 3 * randn(1),-psi_inf(11)),psi_inf(11));
           min(max(psi_inf(12) / 3 * randn(1),-psi_inf(12)),psi_inf(12));];

%     Adds uncertainty to state for computing Lfhs, etc
    q = q + psi;

    % System Parameters
    g           = 9.81;
    tau1        = tau;                                % Safe Following Time (sec)
    mu          = 5;
    gamma1      = 1 + 1/mu;
    gamma2      = 1 - 1/mu;

    u_max       = U * 0.25 * M * g;
    w_max       = U * pi / 18;
    phi_inf     = phi_scale * [w_max; u_max];
    car_width   = 2.27; % m
    car_length  = 5.05; % m
    ls          = lane - car_width/2;
    edge1       = car_width/2;
    edge2       = 2*lane - car_width/2;

    % Formulate Dynamics 
    fq = [  q(4) * cos(q(3));
            q(4) * sin(q(3));
                           0;
                           0;
            q(8) * cos(q(7));
            q(8) * sin(q(7));
                           0;
                           0;
          q(12) * cos(q(11));
          q(12) * sin(q(11));
                           0;
                           0];

    gq = [0     0; % u = [w v] where w is orientation control, v is lin vel control
          0     0;
          1     0;
          0 1 / M;
          0     0;
          0     0;
          0     0;
          0     0;
          0     0;
          0     0;
          0     0;
          0     0];

    % Coordinate Transformation and Associated Dynamics
    qbar = q - qd;

    xdot_d     = qd(4);
    ydot_d     = 0;
    vdot_d     = 0;
    thetadot_d = ((xdot_d - q(4)*cos(q(3)))*qbar(2) - (ydot_d - ...
                   q(3)*sin(q(4)))*qbar(1)) / (qbar(1) ^ 2 + qbar(2) ^ 2);

    fqbar = [  q(4)*cos(q(3)) - xdot_d;
               q(4)*sin(q(3)) - ydot_d;
                       -1 * thetadot_d;
                           -1 * vdot_d;
               q(8)*cos(q(7)) - xdot_d;
               q(8)*sin(q(7)) - ydot_d;
                       -1 * thetadot_d;
                           -1 * vdot_d;
             q(12)*cos(q(11)) - xdot_d;
             q(12)*sin(q(11)) - ydot_d;
                       -1 * thetadot_d;
                           -1 * vdot_d;
            ];

    gqbar = gq;

    % dx, dy - for notation simplification
    dx  = [q(5) - q(1); q(9 ) - q(1)] * 1;
    dy  = [q(6) - q(2); q(10) - q(2)] * 1;
    vf  =  q(4)*cos(q(3));
    tau = tau1;

    % Formulate Lyapunov Function(s)
    % These gains are somewhat in flux, still some tuning required
    kx  = 1 / (60^2);
    ky  = 100;
    kt  = 400;
    kv  = 1;
    kxv = 1000*sqrt(kx)*sqrt(kv);
    kyt = 100*sqrt(ky)*sqrt(kt);

    hg  = K * (-1 + kx*qbar(1)^2 + ky*qbar(2)^2 + kt*qbar(3)^2 + kv*qbar(4)^2 + (kxv*qbar(1)*qbar(4))^2 + (kyt*qbar(2)*qbar(3))^2);
    dhg = K * [2*kx*qbar(1) +  2*kxv*qbar(1)*qbar(4)^2;
               2*ky*qbar(2) +  2*kyt*qbar(2)*qbar(3)^2; 
               2*kyt*qbar(2)^2*qbar(3) + 2*kt*qbar(3); 
               2*kxv*qbar(1)^2*qbar(4) + 2*kv*qbar(4)]';
    sum_dhg = sum(dhg);

    % Formulate Barrier Function(s)
    hs  = zeros(3,1);
    dhs = zeros(3,4);

    % Remain on road OR Remain at desired Y
    % Remain at desired Y is an attempt to force the ego vehicle to employ the
    % longitudinal acceleration control to obey safety rather than swerve
    % to avoid - makes sense that default case would be to swerve to avoid,
    % as the magnitude of w_max (angular vel) is less than the magnitude of u_max
    % (longitudinal accel)
    epsilon = 0.1;
    real_edge1 = edge1;
    real_edge2 = edge2;
    if abs(q(2) - qd(2)) < epsilon
        edge1 = (qd(2) - (epsilon ));
        edge2 = (qd(2) + (epsilon ));
        khs   = 100; % This is just a multiplier used for debugging purposes
    else
        khs = 1;
    end
    
    % The 'edges' are the edge of the road minus the distance the ego
    % vehicle would travel in the y-direction if employing its maximum
    % angular control in order to avoid driving off the road
    real_e1 = real_edge1 + q(4)*w_max*(1 - cos(q(3)));
    real_e2 = real_edge2 - q(4)*w_max*(1 - cos(q(3)));
    e1 = edge1 + q(4)*w_max*(1 - cos(q(3)));
    e2 = edge2 - q(4)*w_max*(1 - cos(q(3)));

    % Safety wrt Road or y_desired
    hs(1)    = khs * (q(2) - e1) * (q(2) - e2);
    dhs(1,:) = khs * [0;
                2*q(2) - edge1 - edge2;
                q(4) * w_max * sin(q(3)) * ((q(2) - e1) - (q(2) - e2));
                w_max * (1 - cos(q(3))) * ((q(2) - e1) - (q(2) - e2));
               ];
    % % Obey Speed Limit
    % hs(2)    = 10 * (q(4) - speed_limit);
    % dhs(2,:) = [0;
    %             0;
    %             0;
    %             1;
    %            ] * 10;

    % Safety wrt lead vehicle - ellipse centered on lead vehicle
    hs(3)    = 1 - (dx(1) / (vf * tau(1) + car_length)) ^ 2 - (dy(1) / ls) ^ 2;
    dhs(3,:) = [-2 * dx(1) / (vf * tau(1) + car_length) ^ 2;
                -2 * dy(1) / ls ^ 2;
                2 * (dx(1) / (q(4) * tau(1) + car_length)) ^ 2 / cos(q(3)) ^ 3;
                2 * (dx(1) / (tau(1) * cos(q(3)) + car_length)) ^ 2 / q(4) ^ 3;];
    sum_dhs = 0;
    if max(hs(1),hs(3)) >= 0
        if hs(1) > hs(3)
            idx = 1;
        else
            idx = 3;
        end
        sum_dhs = sum(dhs(idx));
    end

    % Compute Lie Derivatives
    Lfhg = dhg * fqbar(1:4);
    Lghg = dhg * gqbar(1:4,:);
    Lfhs = dhs * fqbar(1:4);
    Lghs = dhs * gqbar(1:4,:);

    % Formulate QP
    % Standard QP Formulation - J = ||u||^2 + p1*d1^2 + p2*d2^2 + q1*d1
    Q = [1 0  0  0  0;
         0 1  0  0  0;
         0 0 p1  0  0;
         0 0  0 p2  0;
         0 0  0  0 p3;];

    p_vector = [0; 0; q1; 0; 0];

    % CBF Conditions
    G0 = [Lghs(:,1) Lghs(:,2) zeros(size(hs)) [hs(1); 0; 0] [0; 0; hs(3)]];
    h0 = -1*Lfhs; 

    a1 = pi*mu/(2*T);
    
    % FxTS CLF Conditions
    Gc = [Lghg(:,1) Lghg(:,2) -1 zeros(size(hg)) zeros(size(hg))];
    hc = -1*Lfhg - a1*max(0, hg)^gamma1 - a1*max(0, hg)^gamma2;
    
    G = [
          1  0 0 0 0;
         -1  0 0 0 0;
          0  1 0 0 0;
          0 -1 0 0 0;
        ];

    h_vector = [
                       w_max;
                       w_max;
                       u_max; 
                       u_max;
               ];

    % Combine Safety, Convergence, and Input Constraints
    G = [G0; Gc; G];
    h_vector = [h0; hc; h_vector];
    
    dx  = [q_real(5) - q_real(1); q_real(9 ) - q_real(1)] * 1;
    dy  = [q_real(6) - q_real(2); q_real(10) - q_real(2)] * 1;
    vf  =  q_real(4)*cos(q_real(3));
    % Report CBF1 wrt Road, in case that hs1 was defined wrt yd
    hs(1) = (q_real(2) - real_e1) * (q_real(2) - real_e2);
    hs(3) = 1 - (dx(1) / (vf * tau(1) + car_length)) ^ 2 - (dy(1) / ls) ^ 2;

    % Solve Quadratic Programming Problem
    sol = quadprog(Q,p_vector,G,h_vector,[],[],[],[],[],options);
    if isempty(sol)
        q
    end
%     % @KUNAL This is my proposed solution for Infeasible QP
%     % QP was considered infeasible - numerical problem
%     N = 100;
%     while exitflag == -2
%         G_d1 = [0 0 0 0 -1  0;
%                 0 0 0 0  0 -1];
%         h_d1 = [-N; 0];
%         G = [G; G_d1];
%         h_vector = [h_vector; h_d1];
%         
%         [sol,fval,exitflag] = quadprog(Q,p_vector,G,h_vector,[],[],[],[],[],options);
%         N = N * 10;
%     end
        
    w      = sol(1);
    u      = sol(2);
    delta1 = sol(3);
    
%     % This is for debugging the infeasible case, ignore
%     if abs(sol(1)) > w_max || abs(sol(2)) > u_max
%         exitflag
%         disp('control constraint violated')
%     end

    % Update Dynamics

    % Generate Input Disturbance (Zero-Mean Gaussian)
    phi = [min(max(phi_inf(1) / 3 * randn(1),-phi_inf(1)),phi_inf(1));
           min(max(phi_inf(2) / 3 * randn(1),-phi_inf(2)),phi_inf(2))];

    % Update Dynamics
    qdot  = fq_real + gq * ([w; u] + phi);
    new_q = q_real + dt*qdot;
    new_u = [w u];

    % Send back flag to main script if safety violated
    if max(hs) > 0
        disp('Safety was Violated!')
        safety_violation = true;
    else
        safety_violation = false;
    end

end

function [T_ub] = compute_T_worst(T,c3)
    I = zeros(length(T),1);
    mu = 5;
    for t = 1:length(T)
        c1 = mu*pi/(2*T(t));
        c2 = c1;
        k1 = sqrt((4*c1*c2 - c3^2) / (4*c1^2));
        k2 = -c3 / sqrt(4*c1*c2 - c3^2);
        Vbar = c3 / (2*sqrt(c1*c2));
        
        I(t) = mu / (c1*k1) * (pi/2 - atan(k2));
    end
    
    T_ub = sum(I);
end
