clc; clear; close all;

%% More Plots
color = repmat(['b','k','g','r','m','c'],1,5);
lw    = 6.0;

% Plotting params
theta = 0:2*pi/101:2*pi;
RR    = 0.6;

% Physical Params
L = 0.85;

%% Plot 1
clf; close all;
timestepN = 1044;
dtimestep  = 200;
nAgents    = 4;

load('datastore/dynamic_bicycle_rdrive/real_ones/ff_cbf_4intersection_rpca.mat')
% figure('DefaultAxesFontSize',big_font_size);
% title('XY Trajectories'); xlabel('X (m)'); ylabel('Y (m)');
hold on;
for oo = 1:length(obstacles)
    plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2,'HandleVisibility','off')
end
% for ii=1:1:nAgents
%     cx1 = x(timestepN,ii,1) + L/2*cos(x(timestepN,ii,3));
%     cy1 = x(timestepN,ii,2) + L/2*sin(x(timestepN,ii,3));
%     cx2 = x(timestepN,ii,1) - L/2*cos(x(timestepN,ii,3));
%     cy2 = x(timestepN,ii,2) - L/2*sin(x(timestepN,ii,3));
% 
%     ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
%     ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);
% 
%     if ii < 4
%         ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
%         ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);
%     else
%         ox1a = cx1 + RR*wrapToPi(theta)/pi; oy1a = cy1 + RR*wrapToPi(theta)/pi;
%         ox1b = cx1 - RR*wrapToPi(theta)/pi; oy1b = cy1 + RR*wrapToPi(theta)/pi;
%         ox2a = cx2 + RR*wrapToPi(theta)/pi; oy2a = cy2 + RR*wrapToPi(theta)/pi;
%         ox2b = cx2 - RR*wrapToPi(theta)/pi; oy2b = cy2 + RR*wrapToPi(theta)/pi;
%         
%         ox1  = [ox1a ox1b];
%         oy1  = [oy1a oy1b];
%         ox2  = [ox2a ox2b];
%         oy2  = [oy2a oy2b];
%     end
%     
%     plot(ox1, oy1,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);
%     plot(ox2, oy2,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);
% 
% end
% 
% for ttt = 1:dtimestep:timestepN
%     for jj = 1:nAgents
%         plot(x(ttt,jj,1),x(ttt,jj,2),'o','Color',color(jj),'MarkerSize',mksz,'LineWidth',(lw-3))
%     end
% end

load('datastore/dynamic_bicycle_rdrive/real_ones/scbf_4intersection_robust.mat')
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
maxXdim = 15;
maxYdim = 12;
title('Standard CBF','FontSize',big_font_size); 
xlabel('X (m)','FontSize',big_font_size); 
ylabel('Y (m)','FontSize',big_font_size);

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

    if ii < 4
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


% load('datastore/dynamic_bicycle_rdrive/real_ones/scbf_4intersection_pcca.mat')
% lw = 6.0;
% for ii=1:1:nAgents
%     if ii == 2 || ii == 3
%         continue
%     end
%     cx1 = x(timestepN,ii,1) + L/2*cos(x(timestepN,ii,3));
%     cy1 = x(timestepN,ii,2) + L/2*sin(x(timestepN,ii,3));
%     cx2 = x(timestepN,ii,1) - L/2*cos(x(timestepN,ii,3));
%     cy2 = x(timestepN,ii,2) - L/2*sin(x(timestepN,ii,3));
% 
%     ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
%     ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);
% 
%     if ii < 4
%         ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
%         ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);
%     else
%         ox1a = cx1 + RR*wrapToPi(theta)/pi; oy1a = cy1 + RR*wrapToPi(theta)/pi;
%         ox1b = cx1 - RR*wrapToPi(theta)/pi; oy1b = cy1 + RR*wrapToPi(theta)/pi;
%         ox2a = cx2 + RR*wrapToPi(theta)/pi; oy2a = cy2 + RR*wrapToPi(theta)/pi;
%         ox2b = cx2 - RR*wrapToPi(theta)/pi; oy2b = cy2 + RR*wrapToPi(theta)/pi;
%         
%         ox1  = [ox1a ox1b];
%         oy1  = [oy1a oy1b];
%         ox2  = [ox2a ox2b];
%         oy2  = [oy2a oy2b];
%     end
%     
%     plot(ox1, oy1,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);
%     plot(ox2, oy2,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);
% 
% end
% for ttt = 1:dtimestep:timestepN
%     for jj = 1:nAgents
%         if ii == 2 || ii == 3
%             continue
%         end
%         plot(x(ttt,jj,1),x(ttt,jj,2),'o','Color',color(jj),'MarkerSize',mksz,'LineWidth',(lw-3))
%     end
% end

legend show
set(gca,'FontSize',big_font_size)


txt = strcat('t = ',num2str(timestepN/1000),' sec');
text(5,5,txt,'FontSize',big_font_size)
axis([-maxXdim maxXdim -maxYdim maxYdim]);

set(gcf,'renderer','painters')
set(gcf, 'Position', get(0, 'Screensize'));
set(gca,'DataAspectRatio',[1 1 1],...
        'PlotBoxAspectRatio',[1 1 1])
saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T1_scbf_robust.eps','epsc')

%% Plot 2
clf; close all;
timestepN = 1100;
dtimestep  = 200;
nAgents    = 4;

hold on;
for oo = 1:length(obstacles)
    plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2,'HandleVisibility','off')
end


load('datastore/dynamic_bicycle_rdrive/real_ones/scbf_4intersection_pcca.mat')
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
title('Standard CBF w/ PCCA','FontSize',big_font_size); 
xlabel('X (m)','FontSize',big_font_size); 
ylabel('Y (m)','FontSize',big_font_size);
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
    cx1 = x(timestepN,ii,1) + L/2*cos(x(timestepN,ii,3));
    cy1 = x(timestepN,ii,2) + L/2*sin(x(timestepN,ii,3));
    cx2 = x(timestepN,ii,1) - L/2*cos(x(timestepN,ii,3));
    cy2 = x(timestepN,ii,2) - L/2*sin(x(timestepN,ii,3));

    ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
    ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);

    if ii < 4
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


txt = strcat('t = ',num2str(timestepN/1000),' sec');
text(5,5,txt,'FontSize',big_font_size)
axis([-maxXdim maxXdim -maxYdim maxYdim]);

set(gcf,'renderer','painters')
set(gcf, 'Position', get(0, 'Screensize'));
set(gca,'DataAspectRatio',[1 1 1],...
        'PlotBoxAspectRatio',[1 1 1])
saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T1_scbf_pcca.eps','epsc')


%% Plot 3
clf; close all;
timestepN  = 5000;
timestepS1 = 1;
timestepS1 = 1150;
timestepS1 = 2000;
timestepS1 = 4000;
% timestepS1 = 3000;
% timestepS1 = 4000;
dtimestep  = 200;
nAgents    = 4;

hold on;
for oo = 1:length(obstacles)
    plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2,'HandleVisibility','off')
end


load('datastore/dynamic_bicycle_rdrive/real_ones/ff_cbf_4intersection_rpca.mat')
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
maxXdim = 15;
maxYdim = 12;
% title('FF-CBF w/ rPCA','FontSize',big_font_size); 
xlabel('X (m)','FontSize',big_font_size); 
ylabel('Y (m)','FontSize',big_font_size);

for ttt = 1:dtimestep/10:timestepN
    for jj = 1:nAgents
        if ttt < timestepS1
            if ttt == 1 && jj == 2
                plot(x(ttt,jj,1),x(ttt,jj,2),'*','Color',color(jj),'MarkerSize',mksz+5,'LineWidth',(lw-4),'DisplayName','Past Path')
            else
                plot(x(ttt,jj,1),x(ttt,jj,2),'*','Color',color(jj),'MarkerSize',mksz+5,'LineWidth',(lw-4),'HandleVisibility','off')
            end
        elseif mod(ttt,1) == 0 || abs(ttt - timestepS1) <= dtimestep/10
            if abs(ttt - timestepS1) <= dtimestep/10 && jj == 2
                plot(x(ttt,jj,1),x(ttt,jj,2),'o','Color',color(jj),'MarkerSize',mksz-3,'LineWidth',(lw-4),'DisplayName','Future Path')
            else
                plot(x(ttt,jj,1),x(ttt,jj,2),'o','Color',color(jj),'MarkerSize',mksz-3,'LineWidth',(lw-4),'HandleVisibility','off')
            end
        end
    end
end

for ii=1:1:nAgents
    cx1 = x(timestepS1,ii,1) + L/2*cos(x(timestepS1,ii,3));
    cy1 = x(timestepS1,ii,2) + L/2*sin(x(timestepS1,ii,3));
    cx2 = x(timestepS1,ii,1) - L/2*cos(x(timestepS1,ii,3));
    cy2 = x(timestepS1,ii,2) - L/2*sin(x(timestepS1,ii,3));

    ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
    ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);

    if ii < 4
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
%     if ii == 2
%         plot(ox1, oy1,'Color',color(ii),'Linewidth',lw,'DisplayName','Coop')%,'MarkerSize',mksz);
%         plot(ox2, oy2,'Color',color(ii),'Linewidth',lw,'HandleVisibility','off')%,'MarkerSize',mksz);
%     else
%         plot(ox1, oy1,'Color',color(ii),'Linewidth',lw,'HandleVisibility','off')%,'MarkerSize',mksz);
%         plot(ox2, oy2,'Color',color(ii),'Linewidth',lw,'HandleVisibility','off')%,'MarkerSize',mksz);
%     end

end

legend show
set(gca,'FontSize',big_font_size)

txt = strcat('t = ',num2str(timestepS1/1000),' sec');
text(5,5,txt,'FontSize',big_font_size)
axis([-maxXdim maxXdim -maxYdim maxYdim]);

set(gcf,'renderer','painters')
set(gcf, 'Position', get(0, 'Screensize'));
set(gca,'DataAspectRatio',[1 1 1],...
        'PlotBoxAspectRatio',[1 1 1])
% saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T0_ffcbf_rpca.eps','epsc')
% saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T1_ffcbf_rpca.eps','epsc')
% saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T2_ffcbf_rpca.eps','epsc')
saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/XY_Trajectories_T3_ffcbf_rpca.eps','epsc')


%% Control Plots rPCA
clf; close all;
timestepN  = 5000;
timestepS1 = 1;
% timestepS1 = 1650;
% timestepS1 = 3000;
timestepS1 = 4000;
dtimestep  = 200;
nAgents    = 4;

hold on;
% for oo = 1:length(obstacles)
%     plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2)
% end


load('datastore/dynamic_bicycle_rdrive/real_ones/ff_cbf_4intersection_rpca.mat')
hold on
lw = 6.0;
mksz  = 10.0;
big_font_size = 36;
subplot(211)
title('FF-CBF w/ rPCA','FontSize',big_font_size); 
% xlabel('t (sec)','FontSize',big_font_size); 
ylabel('\omega (rad/s)','FontSize',big_font_size);
ttt = 1:1:timestepN;

hold on
for jj = 1:nAgents
    plot(ttt/1000,u(ttt,jj,1),'LineWidth',lw,'Color',color(jj))
end
hold off
set(gca,'FontSize',big_font_size)
legend('Coop1','Coop2','Coop3','Noncoop1')


subplot(212)
xlabel('t (sec)','FontSize',big_font_size); 
ylabel('a (m/s^2)','FontSize',big_font_size);

hold on
for jj = 1:nAgents
    plot(ttt/1000,u(ttt,jj,2),'LineWidth',lw,'Color',color(jj))
end
hold off;
set(gca,'FontSize',big_font_size)
legend('Coop1','Coop2','Coop3','Noncoop1')


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
saveas(gcf,'datastore/dynamic_bicycle_rdrive/real_ones/Controls_T3_ffcbf_rpca.eps','epsc')

%% Plot 2
timestep_n = 200;
figure('DefaultAxesFontSize',big_font_size);
title('XY Trajectories'); xlabel('X (m)'); ylabel('Y (m)');
hold on;
for oo = 1:length(obstacles)
    plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2)
end
for ii=1:1:nAgents
    cx1 = x(timestep_n,ii,1) + L/2*cos(x(timestep_n,ii,3));
    cy1 = x(timestep_n,ii,2) + L/2*sin(x(timestep_n,ii,3));
    cx2 = x(timestep_n,ii,1) - L/2*cos(x(timestep_n,ii,3));
    cy2 = x(timestep_n,ii,2) - L/2*sin(x(timestep_n,ii,3));

    ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
    ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);

    if ii < 4
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
    
    plot(ox1, oy1,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);
    plot(ox2, oy2,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);

end

for ttt = 100:10:timestep_n
    for jj = 1:nAgents
        plot(x(ttt,jj,1),x(ttt,jj,2),'o','Color',color(jj),'MarkerSize',mksz,'LineWidth',(lw-3))
    end
end

txt = strcat('t = ',num2str(2.0),' sec');
text(10,10,txt,'FontSize',big_font_size)
axis([-maxXdim maxXdim -maxYdim maxYdim]);
hold off

set(gcf,'renderer','painters')
set(gcf, 'Position', get(0, 'Screensize'));
saveas(gcf,'datastore/dynamic_bicycle_rdrive/money_sets/XY_Trajectories_T2.eps','epsc')



% Plot 3
timestep_n = 350;
figure('DefaultAxesFontSize',big_font_size);
title('XY Trajectories'); xlabel('X (m)'); ylabel('Y (m)');
hold on;
for oo = 1:length(obstacles)
    plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2)
end
for ii=1:1:nAgents
    cx1 = x(timestep_n,ii,1) + L/2*cos(x(timestep_n,ii,3));
    cy1 = x(timestep_n,ii,2) + L/2*sin(x(timestep_n,ii,3));
    cx2 = x(timestep_n,ii,1) - L/2*cos(x(timestep_n,ii,3));
    cy2 = x(timestep_n,ii,2) - L/2*sin(x(timestep_n,ii,3));

    ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
    ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);

    if ii < 4
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
    
    plot(ox1, oy1,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);
    plot(ox2, oy2,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);

end

for ttt = 200:10:timestep_n
    for jj = 1:nAgents
        plot(x(ttt,jj,1),x(ttt,jj,2),'o','Color',color(jj),'MarkerSize',mksz,'LineWidth',(lw-3))
    end
end

txt = strcat('t = ',num2str(3.5),' sec');
text(10,10,txt,'FontSize',big_font_size)
axis([-maxXdim maxXdim -maxYdim maxYdim]);
hold off

set(gcf,'renderer','painters')
set(gcf, 'Position', get(0, 'Screensize'));
saveas(gcf,'datastore/dynamic_bicycle_rdrive/money_sets/XY_Trajectories_T3.eps','epsc')
