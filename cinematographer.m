function cinematographer(dt,x,dyn_mode,obstacles,filename)



color = repmat(['b','k','g','r','m','c'],1,5);
lw    = 3.0;
mksz  = 12.0;
maxsteps = size(x,1);
nAgents = size(x,2);
maxXdim = 25;
maxYdim = 25;

% Plotting params
theta = 0:2*pi/101:2*pi;
RR    = 0.6;
RR    = 2.0; % This corresponds to a physical radius of 1 = RR/2

% Physical Params
L = 0.85;
L = 0.0;
% L = 1.0;

% Make the movie object
mov = struct('cdata', [], 'colormap', []);

% vidObj = VideoWriter(filename, 'MPEG-4'); % Does not work on Ubuntu
vidObj = VideoWriter(strcat(filename,'.avi')); % Works on Ubuntu

% Frame rate. Keep this at 24 if you don't care.
FR = 30;
vidObj.FrameRate = FR;
open(vidObj)

figure(100);

% The first two numbers in the "position" matrix are the (x,y) position of the lower
% left corner of the plot window. The last two numbers are the (x,y) position
% of the upper right corner.
position = [100 50 700 700];
set(gcf, 'Position', position)

ax = gca();

hold on
for oo = 1:length(obstacles)
    plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2)
end
for ii=1:1:nAgents
    if strcmp(dyn_mode,'double_integrator')
        th = atan2(x(1,ii,4),x(1,ii,3));
        cx1 = x(1,ii,1);
        cy1 = x(1,ii,2);
        cx2 = x(1,ii,1);
        cy2 = x(1,ii,2);
    else
        cx1 = x(1,ii,1) + L/2*cos(x(1,ii,3));
        cy1 = x(1,ii,2) + L/2*sin(x(1,ii,3));
        cx2 = x(1,ii,1) - L/2*cos(x(1,ii,3));
        cy2 = x(1,ii,2) - L/2*sin(x(1,ii,3));
    end

    if ii < 5
        ox1 = cx1 + (RR/2)*cos(theta); oy1 = cy1 + (RR/2)*sin(theta);
        ox2 = cx2 + (RR/2)*cos(theta); oy2 = cy2 + (RR/2)*sin(theta);
    else
        ox1a = cx1 + (RR/2)*wrapToPi(theta)/pi; oy1a = cy1 + (RR/2)*wrapToPi(theta)/pi;
        ox1b = cx1 - (RR/2)*wrapToPi(theta)/pi; oy1b = cy1 + (RR/2)*wrapToPi(theta)/pi;
        ox2a = cx2 + (RR/2)*wrapToPi(theta)/pi; oy2a = cy2 + (RR/2)*wrapToPi(theta)/pi;
        ox2b = cx2 - (RR/2)*wrapToPi(theta)/pi; oy2b = cy2 + (RR/2)*wrapToPi(theta)/pi;
        
        ox1  = [ox1a ox1b];
        oy1  = [oy1a oy1b];
        ox2  = [ox2a ox2b];
        oy2  = [oy2a oy2b];
    end
    
    pp(ii) = plot(ax,ox1,oy1,'Color',color(ii),'Linewidth',lw);

%     plot(ox1, oy1,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);
%     plot(ox2, oy2,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);

end
set(0,'CurrentFigure',100);
set(ax, 'XLimMode', 'manual', 'YLimMode', 'manual');
txt = strcat('t = ',num2str(0.0),' sec');
text(10,10,txt,'FontSize',14)
axis([-maxXdim maxXdim -maxYdim maxYdim]);
% hold off
drawnow update

% ax = gca();
mov= getframe(gcf);

% The next few lines make the first frame of the movie play for 2 seconds
first_frame_pause_secs = 1;
for k=1:1:first_frame_pause_secs*FR
    writeVideo(vidObj,mov)
%     writeVideo(vidObj,getframe(ax))
end

% This main loop draws the plots for each remaining time step and saves
%   each plot to a frame of the movie.

% If the movie is going too slow, you can increase the "stride" value, e.g.
%   put "for j = 1:10:maxsteps" or some other number in the middle.
for tt=1:(1/(50*dt)):maxsteps
% for tt=(maxsteps-100):(1/(50*dt)):maxsteps

%     clf
%     set(gcf, 'Position', position)
%     hold on
%     for oo = 1:length(obstacles)
%         plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2)
%     end
    set(0,'CurrentFigure',100);
    for ii=1:1:nAgents
        if strcmp(dyn_mode,'double_integrator')
            th = atan2(x(1,ii,4),x(1,ii,3));
            cx1 = x(tt,ii,1);
            cy1 = x(tt,ii,2);
            cx2 = x(tt,ii,1);
            cy2 = x(tt,ii,2);
        else
            cx1 = x(tt,ii,1) + L/2*cos(x(1,ii,3));
            cy1 = x(tt,ii,2) + L/2*sin(x(1,ii,3));
            cx2 = x(tt,ii,1) - L/2*cos(x(1,ii,3));
            cy2 = x(tt,ii,2) - L/2*sin(x(1,ii,3));
        end

        if ii < 5
            ox1 = cx1 + (RR/2)*cos(theta); oy1 = cy1 + (RR/2)*sin(theta);
            ox2 = cx2 + (RR/2)*cos(theta); oy2 = cy2 + (RR/2)*sin(theta);
        else
%             ox1 = cx1 + sqrt(2)*RR*wrapToPi(theta)/pi; oy1 = cy1 + sqrt(2)*RR*wrapToPi(theta)/pi;
%             ox2 = cx2 + sqrt(2)*RR*wrapToPi(theta)/pi; oy2 = cy2 + sqrt(2)*RR*wrapToPi(theta)/pi;
            
            ox1a = cx1 + (RR/2)*wrapToPi(theta)/pi; oy1a = cy1 + (RR/2)*wrapToPi(theta)/pi;
            ox1b = cx1 - (RR/2)*wrapToPi(theta)/pi; oy1b = cy1 + (RR/2)*wrapToPi(theta)/pi;
            ox2a = cx2 + (RR/2)*wrapToPi(theta)/pi; oy2a = cy2 + (RR/2)*wrapToPi(theta)/pi;
            ox2b = cx2 - (RR/2)*wrapToPi(theta)/pi; oy2b = cy2 + (RR/2)*wrapToPi(theta)/pi;

            ox1  = [ox1a ox1b];
            oy1  = [oy1a oy1b];
            ox2  = [ox2a ox2b];
            oy2  = [oy2a oy2b];
        end

%         pp(ii) = plot(ax,ox1,oy1,'Color',color(ii),'Linewidth',lw);
        set(pp(ii),'XData',ox1,'YData',oy1);
%         plot(ox1, oy1,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);
%         plot(ox2, oy2,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);

    %     plot(x(tt,ii,1), x(tt,ii,2),'o','Color',color(ii),'Linewidth',lw,'MarkerSize',mksz);

    end
    txt = strcat('t = ',num2str(round(tt * dt,2)),' sec');
    delete(findobj(gca,'Type','Text'))
    text(10,10,txt,'FontSize',14)
    axis([-maxXdim maxXdim -maxYdim maxYdim]);

    hold off

    drawnow update
    mov= getframe(gcf);
    writeVideo(vidObj,mov)
    clear mov
    mov = struct('cdata', [], 'colormap', []);
end
%Close the file when you're done
close(vidObj)

end