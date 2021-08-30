function cinematographer(dt,x,obstacles,filename)


color = ['b','g','m','k','r','y'];
lw    = 3.0;
mksz  = 12.0;
maxsteps = size(x,1);
nAgents = size(x,2);
maxXdim = 30;
maxYdim = 30;

% Plotting params
theta = 0:0.2:2*pi;
RR    = 0.6;

% Physical Params
L = 1;

% Make the movie object
mov = struct('cdata', [], 'colormap', []);

vidObj = VideoWriter(filename, 'MPEG-4');

% Frame rate. Keep this at 24 if you don't care.
FR = 30;
vidObj.FrameRate = FR;
open(vidObj)

% Specify images
I1         = imread('images/car1.png');
marker{1}  = I1;
I2         = imread('images/car2.png');
marker{2}  = imrotate(I2,pi);
I3         = imread('images/car3.png');
marker{3}  = imrotate(I3,-pi/2);
markersize = [1,1]; % The size of marker is expressed in axis units, NOT in pixels

figure(1);
clf

% The first two numbers in the "position" matrix are the (x,y) position of the lower
% left corner of the plot window. The last two numbers are the (x,y) position
% of the upper right corner.
position = [100 50 700 700];
set(gcf, 'Position', position)

hold on
for oo = 1:length(obstacles)
    plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2)
end
for ii=1:1:nAgents
    cx1 = x(1,ii,1) + L/2*cos(x(1,ii,3));
    cy1 = x(1,ii,2) + L/2*sin(x(1,ii,3));
    cx2 = x(1,ii,1) - L/2*cos(x(1,ii,3));
    cy2 = x(1,ii,2) - L/2*sin(x(1,ii,3));

    ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
    ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);

%     plot(x(1,ii,1), x(1,ii,2),'o','Color',color(ii),'Linewidth',lw,'MarkerSize',mksz);
    if ii < 4
        mark = 'o';
    else
        mark = 'x';
    end
    plot(ox1, oy1, mark,'Color',color(ii),'Linewidth',lw,'MarkerSize',mksz);
    plot(ox2, oy2, mark,'Color',color(ii),'Linewidth',lw,'MarkerSize',mksz);

end
txt = strcat('t = ',num2str(0.0),' sec');
text(10,10,txt,'FontSize',14)
axis([-maxXdim maxXdim -maxYdim maxYdim]);
hold off
drawnow update
mov= getframe(gcf);

% The next few lines make the first frame of the movie play for 2 seconds
first_frame_pause_secs = 2;
for k=1:1:first_frame_pause_secs*FR
    writeVideo(vidObj,mov)
end

% This main loop draws the plots for each remaining time step and saves
%   each plot to a frame of the movie.

% If the movie is going too slow, you can increase the "stride" value, e.g.
%   put "for j = 1:10:maxsteps" or some other number in the middle.
for tt=1:(1/(10*dt)):maxsteps

    clf
    set(gcf, 'Position', position)
    hold on
    for oo = 1:length(obstacles)
        plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2)
    end
    for ii=1:1:nAgents
        cx1 = x(tt,ii,1) + L/2*cos(x(tt,ii,3));
        cy1 = x(tt,ii,2) + L/2*sin(x(tt,ii,3));
        cx2 = x(tt,ii,1) - L/2*cos(x(tt,ii,3));
        cy2 = x(tt,ii,2) - L/2*sin(x(tt,ii,3));

        if ii < 4
            ox1 = cx1 + RR*cos(theta); oy1 = cy1 + RR*sin(theta);
            ox2 = cx2 + RR*cos(theta); oy2 = cy2 + RR*sin(theta);
        else
            ox1 = cx1 + sqrt(2)*RR*wrapToPi(theta)/pi; oy1 = cy1 + sqrt(2)*RR*wrapToPi(theta)/pi;
            ox2 = cx2 + sqrt(2)*RR*wrapToPi(theta)/pi; oy2 = cy2 + sqrt(2)*RR*wrapToPi(theta)/pi;
        end


        plot(ox1, oy1,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);
        plot(ox2, oy2,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);

    %     plot(x(tt,ii,1), x(tt,ii,2),'o','Color',color(ii),'Linewidth',lw,'MarkerSize',mksz);

    end
    txt = strcat('t = ',num2str(round(tt * dt,2)),' sec');
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