load('20200330_CDC_perturbed_case.mat')

%% All Vehicles
e0 = e_all - 1;
e  = max(e0);

figure(4);
p = zeros(nIter+3,1);
colors = ['b' 'r' 'y' 'c' 'm' 'k' 'g' 'b' 'r' 'c'];
i = 2;
l0 = zeros(length(q_rec(i,s:e,1)),1);
l2 = 3*ones(length(q_rec(i,s:e,1)),1);
l3 = 6*ones(length(q_rec(i,s:e,1)),1);

s1 = subplot(3,1,1);
hold on
yyaxis right
i = 1;
p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
i = 4;
p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
i = 7;
p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
i = 9;
p(9) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
i = 3;
p(12) = plot(q_rec(2,s:e,1),l2,'Color','#000000','LineStyle','--','LineWidth',3,'DisplayName','Lane Line');
plot(q_rec(2,s:e,1),l0,'k','LineWidth',4)
plot(q_rec(2,s:e,1),l3,'k','LineWidth',4)
hold off
legend([p(1),p(4),p(7),p(9)],'fontsize',40)
ax = gca;
ax.YAxis(1).FontSize = 0.1;
ax.YAxis(2).FontSize = 40;
ax.YAxis(2).Color = 'k';
ylim([-0.5 6.5])
xlim([xe q_rec(2,e_all(2)-1) + 275])
txt = ['t_p = ' num2str(t_oncoming(1)) ' sec'];
text(-10,5,txt,'fontsize',40)



s2 = subplot(3,1,2);
hold on
yyaxis right
i = 3;
p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
i = 6;
p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
i = 10;
p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
p(13) = plot(q_rec(2,s:e,1),l2,'Color','#000000','LineStyle','--','LineWidth',3,'DisplayName','Lane Line');
plot(q_rec(2,s:e,1),l0,'k','LineWidth',4)
plot(q_rec(2,s:e,1),l3,'k','LineWidth',4)
hold off
legend([p(3),p(6),p(10)],'fontsize',40)
xlim([xe q_rec(2,e_all(2)-1) + 275])
ax = gca;
ax.YAxis(1).FontSize = 0.1;
ax.YAxis(2).FontSize = 40;
ax.YAxis(2).Color = 'k';
ylim([-0.5 6.5])
yyaxis left
ylabel('Transverse Distance, Y (m)','fontsize',60,'Color','k');
txt = ['t_p = ' num2str(t_oncoming(3)) ' sec'];
text(-0.1,0.75,txt,'fontsize',40)


s3 = subplot(3,1,3);
hold on
yyaxis right
i = 2;
p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
i = 5;
p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
i = 8;
p(i) = plot(q_rec(i,s:e_all(i)-1,1),q_rec(i,s:e_all(i)-1,2),'DisplayName',['\phi_{max} = ' num2str(round(phi_inf(i),2))],'LineWidth',3,'LineStyle','-','Color',colors(i));
i = 3;
p(12) = plot(q_rec(2,s:e,1),l2,'Color','#000000','LineStyle','--','LineWidth',3,'DisplayName','Lane Line');
plot(q_rec(2,s:e,1),l0,'k','LineWidth',4)
plot(q_rec(2,s:e,1),l3,'k','LineWidth',4)
hold off
legend([p(2),p(5),p(8)],'fontsize',40)
ax = gca;
ax.YAxis(1).FontSize = 0.1;
ax.YAxis(2).FontSize = 40;
ax.YAxis(2).Color = 'k';
ylim([-0.5 6.5])
xlim([xe q_rec(2,e_all(2)-1) + 275])
xlabel('Longitudinal Distance, X (m)','fontsize',60);
txt = ['t_p = ' num2str(t_oncoming(2)) ' sec'];
text(-10,5,txt,'fontsize',40)



pos1 = get(s1,'Position');
pos1(4) = pos1(4) + 0.07;
set(s1,'Position',pos1);
pos2 = get(s2,'Position');
pos2(4) = pos2(4) + 0.01;
set(s2,'Position',pos2);
pos3 = get(s3,'Position');
pos4(4) = pos3(4) + 0.03;
set(s3,'Position',pos3);