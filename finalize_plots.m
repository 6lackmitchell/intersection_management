clc; clear; close all;

%% More Plots
color = repmat(['b','k','g','r','m','c'],1,5);
lw    = 6.0;

% Plotting params
theta = 0:2*pi/101:2*pi;
RR    = 0.8;

% Physical Params
L = 0;

% Load road geometry
road_file = 'datastore/geometry/road_markings.mat';       
load(road_file)

%% Plot 1
clf; close all;
timestepN = 1000;
dtimestep  = 200;
deadlock   = 184;
nAgents    = 4;

filename = strcat('datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/nominal_cbf/ff_cbf_4MonteCarlo_N1000.mat');
% filename = strcat('datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/ff_cbf_4MonteCarlo_N1000_lookahead5_ffnorv.mat');
% filename = strcat('datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/ff_cbf_4MonteCarlo_N1000_lookahead5_GOAT_RVCBF.mat');


load(filename)
x = trial_data(deadlock).x;
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
maxXdim = 15;
maxYdim = 12;
title('0-CBF','FontSize',big_font_size); 
xlabel('X (m)','FontSize',big_font_size); 
ylabel('Y (m)','FontSize',big_font_size);
hold on;
for oo = 1:length(obstacles)
    plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2,'HandleVisibility','off')
end

for ttt = 1:dtimestep/10:timestepN
    for jj = 1:nAgents
        if ttt == 1 && jj == 2
            plot(x(ttt,jj,1),x(ttt,jj,2),'*','Color',color(jj),'MarkerSize',mksz+5,'LineWidth',(lw-4),'DisplayName','Past Path')
        else
            plot(x(ttt,jj,1),x(ttt,jj,2),'*','Color',color(jj),'MarkerSize',mksz+5,'LineWidth',(lw-4),'HandleVisibility','off')
        end
    end
end
for ii=1:1:nAgents
%     if ii > 1
%         continue
%     end
    cx1 = x(timestepN,ii,1) + L/2*cos(x(timestepN,ii,3));
    cy1 = x(timestepN,ii,2) + L/2*sin(x(timestepN,ii,3));
    cx2 = x(timestepN,ii,1) - L/2*cos(x(timestepN,ii,3));
    cy2 = x(timestepN,ii,2) - L/2*sin(x(timestepN,ii,3));

    ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
    ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);

    if ii < 5
        ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
        ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);
    else
        ox1a = cx1 + RR*wrapToPi(theta)/pi; oy1a = cy1 + RR*wrapToPi(theta)/pi;
        ox1b = cx1 - RR*wrapToPi(theta)/pi; oy1b = cy1 + RR*wrapToPi(theta)/pi;
        ox2a = cx2 + RR*wrapToPi(theta)/pi; oy2a = cy2 + RR*wrapToPi(theta)/pi;
        ox2b = cx2 - RR*wrapToPi(theta)/pi; oy2b = cy2 + RR*wrapToPi(theta)/pi;
        
        ox1  = [ox1a ox1b];
        oy1  = [oy1a oy1b];
        ox2  = [ox2a ox2b];
        oy2  = [oy2a oy2b];
    end
    
    plot(ox1, oy1,'Color',color(ii),'Linewidth',lw,'HandleVisibility','off')%,'MarkerSize',mksz);
    plot(ox2, oy2,'Color',color(ii),'Linewidth',lw,'HandleVisibility','off')%,'MarkerSize',mksz);

end

legend show
set(gca,'FontSize',big_font_size)


txt = strcat('t = ',num2str(timestepN/100),' sec');
text(5,5,txt,'FontSize',big_font_size)
axis([-maxXdim maxXdim -maxYdim maxYdim]);

set(gcf,'renderer','painters')
set(gcf, 'Position', get(0, 'Screensize'));
set(gca,'DataAspectRatio',[1 1 1],...
        'PlotBoxAspectRatio',[1 1 1])
saveas(gcf,'datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/nominal_cbf/deadlock_184.eps','epsc')

%% Plot 2 -- FFCBF
clf; close all;
timestepN = 160;
dtimestep  = 200;
infeasible = 404;
nAgents    = 4;

filename = strcat('datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/ff_cbf_4MonteCarlo_N1000_lookahead5_ffnorv.mat');
% filename = strcat('datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/ff_cbf_4MonteCarlo_N1000_lookahead5_GOAT_RVCBF.mat');

load(filename)
x = trial_data(infeasible).x;
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
maxXdim = 15;
maxYdim = 12;
title('FF-CBF','FontSize',big_font_size); 
xlabel('X (m)','FontSize',big_font_size); 
ylabel('Y (m)','FontSize',big_font_size);
hold on;
for oo = 1:length(obstacles)
    plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2,'HandleVisibility','off')
end

for ttt = 1:dtimestep/10:timestepN
    for jj = 1:nAgents
        if ttt == 1 && jj == 2
            plot(x(ttt,jj,1),x(ttt,jj,2),'*','Color',color(jj),'MarkerSize',mksz+5,'LineWidth',(lw-4),'DisplayName','Past Path')
        else
            plot(x(ttt,jj,1),x(ttt,jj,2),'*','Color',color(jj),'MarkerSize',mksz+5,'LineWidth',(lw-4),'HandleVisibility','off')
        end
    end
end
for ii=1:1:nAgents
%     if ii > 1
%         continue
%     end
    cx1 = x(timestepN,ii,1) + L/2*cos(x(timestepN,ii,3));
    cy1 = x(timestepN,ii,2) + L/2*sin(x(timestepN,ii,3));
    cx2 = x(timestepN,ii,1) - L/2*cos(x(timestepN,ii,3));
    cy2 = x(timestepN,ii,2) - L/2*sin(x(timestepN,ii,3));

    ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
    ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);

    if ii < 5
        ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
        ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);
    else
        ox1a = cx1 + RR*wrapToPi(theta)/pi; oy1a = cy1 + RR*wrapToPi(theta)/pi;
        ox1b = cx1 - RR*wrapToPi(theta)/pi; oy1b = cy1 + RR*wrapToPi(theta)/pi;
        ox2a = cx2 + RR*wrapToPi(theta)/pi; oy2a = cy2 + RR*wrapToPi(theta)/pi;
        ox2b = cx2 - RR*wrapToPi(theta)/pi; oy2b = cy2 + RR*wrapToPi(theta)/pi;
        
        ox1  = [ox1a ox1b];
        oy1  = [oy1a oy1b];
        ox2  = [ox2a ox2b];
        oy2  = [oy2a oy2b];
    end
    
    plot(ox1, oy1,'Color',color(ii),'Linewidth',lw,'HandleVisibility','off')%,'MarkerSize',mksz);
    plot(ox2, oy2,'Color',color(ii),'Linewidth',lw,'HandleVisibility','off')%,'MarkerSize',mksz);

end

legend show
set(gca,'FontSize',big_font_size)


txt = strcat('t = ',num2str(timestepN/100),' sec');
text(5,5,txt,'FontSize',big_font_size)
axis([-maxXdim maxXdim -maxYdim maxYdim]);

set(gcf,'renderer','painters')
set(gcf, 'Position', get(0, 'Screensize'));
set(gca,'DataAspectRatio',[1 1 1],...
        'PlotBoxAspectRatio',[1 1 1])
saveas(gcf,'datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/infeasible_404.eps','epsc')

%% Plot 3 -- RVCBF
clf; close all;
timestepN = 285;
dtimestep  = 200;
success = 404;
nAgents    = 4;

filename = strcat('datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/ff_cbf_4MonteCarlo_N1000_lookahead5_GOAT_RVCBF.mat');

load(filename)
x = trial_data(success).x;
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
maxXdim = 15;
maxYdim = 12;
title('RV-CBF','FontSize',big_font_size); 
xlabel('X (m)','FontSize',big_font_size); 
ylabel('Y (m)','FontSize',big_font_size);
hold on;
for oo = 1:length(obstacles)
    plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2,'HandleVisibility','off')
end

for ttt = 1:dtimestep/10:timestepN
    for jj = 1:nAgents
        if ttt == 1 && jj == 2
            plot(x(ttt,jj,1),x(ttt,jj,2),'*','Color',color(jj),'MarkerSize',mksz+5,'LineWidth',(lw-4),'DisplayName','Past Path')
        else
            plot(x(ttt,jj,1),x(ttt,jj,2),'*','Color',color(jj),'MarkerSize',mksz+5,'LineWidth',(lw-4),'HandleVisibility','off')
        end
    end
end
for ii=1:1:nAgents
%     if ii > 1
%         continue
%     end
    cx1 = x(timestepN,ii,1) + L/2*cos(x(timestepN,ii,3));
    cy1 = x(timestepN,ii,2) + L/2*sin(x(timestepN,ii,3));
    cx2 = x(timestepN,ii,1) - L/2*cos(x(timestepN,ii,3));
    cy2 = x(timestepN,ii,2) - L/2*sin(x(timestepN,ii,3));

    ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
    ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);

    if ii < 5
        ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
        ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);
    else
        ox1a = cx1 + RR*wrapToPi(theta)/pi; oy1a = cy1 + RR*wrapToPi(theta)/pi;
        ox1b = cx1 - RR*wrapToPi(theta)/pi; oy1b = cy1 + RR*wrapToPi(theta)/pi;
        ox2a = cx2 + RR*wrapToPi(theta)/pi; oy2a = cy2 + RR*wrapToPi(theta)/pi;
        ox2b = cx2 - RR*wrapToPi(theta)/pi; oy2b = cy2 + RR*wrapToPi(theta)/pi;
        
        ox1  = [ox1a ox1b];
        oy1  = [oy1a oy1b];
        ox2  = [ox2a ox2b];
        oy2  = [oy2a oy2b];
    end
    
    plot(ox1, oy1,'Color',color(ii),'Linewidth',lw,'HandleVisibility','off')%,'MarkerSize',mksz);
    plot(ox2, oy2,'Color',color(ii),'Linewidth',lw,'HandleVisibility','off')%,'MarkerSize',mksz);

end

legend show
set(gca,'FontSize',big_font_size)


txt = strcat('t = ',num2str(timestepN/100),' sec');
text(5,5,txt,'FontSize',big_font_size)
axis([-maxXdim maxXdim -maxYdim maxYdim]);

set(gcf,'renderer','painters')
set(gcf, 'Position', get(0, 'Screensize'));
set(gca,'DataAspectRatio',[1 1 1],...
        'PlotBoxAspectRatio',[1 1 1])
saveas(gcf,'datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/success_404.eps','epsc')


%% Control Plots rPCA
clf; close all;

subplot(211)
title('RV-CBF Controls','FontSize',big_font_size); 
% xlabel('t (sec)','FontSize',big_font_size); 
ylabel('\omega (rad/s)','FontSize',big_font_size);
ttt = 1:1:timestepN;

u = trial_data(success).u;

hold on
plot(ttt/100,4*pi*ones(length(ttt),1),':','LineWidth',lw,'Color','k')
plot(ttt/100,-4*pi*ones(length(ttt),1),':','LineWidth',lw,'Color','k')
for jj = 1:nAgents
    plot(ttt/100,u(ttt,jj,1),'LineWidth',lw,'Color',color(jj))
end
hold off
set(gca,'FontSize',big_font_size)
legend('','','V1','V2','V3','V4')


subplot(212)
xlabel('t (sec)','FontSize',big_font_size); 
ylabel('a (m/s^2)','FontSize',big_font_size);

hold on
plot(ttt/100,9.81*ones(length(ttt),1),':','LineWidth',lw,'Color','k')
plot(ttt/100,-9.81*ones(length(ttt),1),':','LineWidth',lw,'Color','k')
for jj = 1:nAgents
    plot(ttt/100,u(ttt,jj,2),'LineWidth',lw,'Color',color(jj))
end
hold off;
set(gca,'FontSize',big_font_size)
legend('','','V1','V2','V3','V4')


% txt = strcat('t = ',num2str(timestepS1/1000),' sec');
% text(5,5,txt,'FontSize',big_font_size)
% axis([-maxXdim maxXdim -maxYdim maxYdim]);

set(gcf,'renderer','painters')
set(gcf, 'Position', get(0, 'Screensize'));
% set(gca,'DataAspectRatio',[1 1 1],...
%         'PlotBoxAspectRatio',[1 1 1])
% set(gca,'PlotBoxAspectRatio',[1 1 1])
% saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T0_ffcbf_rpca.eps','epsc')
% saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T1_ffcbf_rpca.eps','epsc')
% saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T2_ffcbf_rpca.eps','epsc')
saveas(gcf,'datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/success_404_controls.eps','epsc')

%% Plot CBFs
clf; close all;

figure();
title('RV-CBF Values','FontSize',big_font_size); 
xlabel('t (sec)','FontSize',big_font_size); 
ylabel('h','FontSize',big_font_size);
ttt = 1:1:timestepN;

u = trial_data(success).u;

H  = zeros(timestepN,6,1);
h0 = zeros(timestepN,6,1);
for tcbf = 1:length(ttt)
    [H(tcbf,:),h0(tcbf,:)] = get_safety_values(squeeze(x(tcbf,:,:)));
end

hold on
plot(ttt/100,zeros(length(ttt),1),'-.','LineWidth',lw,'Color',color(2))
for jj = 1:6
    plot(ttt/100,H(ttt,jj,1),'LineWidth',lw,'Color',color(jj))
    plot(ttt/100,h0(ttt,jj,1),':','LineWidth',lw,'Color',color(jj))
end
hold off
xlim([-0.1 3.3])
ylim([-3 28])
set(gca,'FontSize',big_font_size)
legend('','H(12)','h_0(12)','H(13)','h_0(13)','H(14)','h_0(14)','H(23)','h_0(23)','H(24)','h_0(24)','H(34)','h_0(34)')

set(gcf,'renderer','painters')
set(gcf, 'Position', get(0, 'Screensize'));
% set(gca,'DataAspectRatio',[1 1 1],...
%         'PlotBoxAspectRatio',[1 1 1])
% set(gca,'PlotBoxAspectRatio',[1 1 1])
% saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T0_ffcbf_rpca.eps','epsc')
% saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T1_ffcbf_rpca.eps','epsc')
% saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T2_ffcbf_rpca.eps','epsc')
saveas(gcf,'datastore/robust_virtual/dynamic_bicycle_rdrive_1u/no_backup/input_constraints/rv_cbf/success_404_cbfs.eps','epsc')



%% Safety Helper
function [Ht,H0] = get_safety_values(x)
Na    = 4;
tmax  = 5;

sw = 1.0;

Nc  = factorial(Na-1);
Ht  = zeros(Nc,1);
H0  = zeros(Nc,1);
cc  = 1;
ss  = 1;

% Loop through every scheduled agent for PCCA
for aa = 1:Na
    
    nc  = Na-aa;
    hw  = zeros(nc,1);
    hw0 = zeros(nc,1);
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

        % h and hdot (= Lfh + Lgh*u)
        h0   = dx^2 + dy^2 - (2*sw)^2;
        h    = dx^2 + dy^2 + tau^2*(dvx^2 + dvy^2) + 2*tau*(dx*dvx + dy*dvy) - (2*sw)^2;

        % Robust-Virtual CBF
        a1    = 0.1;
        kh0   = 1;
        H     = h   + a1*(tau-1)*h0^(1/kh0);
    
        % Inequalities: Ax <= b
        hw(dd)          = H;
        hw0(dd)         = h0;

        dd = dd + 1;
        ss = ss + 1;

    end

    Ht(cc:cc+(nc-1))  = hw;
    H0(cc:cc+(nc-1))  = hw0;

    cc = cc + nc;
    
end

end