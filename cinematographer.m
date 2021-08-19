function cinematographer(dt,x,obstacles,filename)


color = ['b','g','m','k','r','y'];
lw    = 3.0;
mksz  = 12.0;
maxsteps = size(x,1);
nAgents = size(x,2);
maxXdim = 10;
maxYdim = 10;

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
    plot(x(1,ii,1), x(1,ii,2),'o','Color',color(ii),'Linewidth',lw,'MarkerSize',mksz);
%     x_low  = x(1,ii,1) - markersize(1)/2; % Left edge of marker
%     x_high = x(1,ii,1) + markersize(1)/2; % Right edge of marker
%     y_low  = x(1,ii,2) - markersize(2)/2; %//Bottom edge of marker
%     y_high = x(1,ii,2) + markersize(2)/2;%//Top edge of marker
% 
%     phi = x(1,ii,3);
%     p0  = [x(1,ii,1); x(1,ii,2)];
%     R   = [cos(phi) sin(phi); -sin(phi) cos(phi)];
%     p1  = [x_high; y_low];
%     p2  = [x_high; y_high];
%     p3  = [x_low; y_high];
%     p4  = [x_low; y_low];
%     
%     p11 = p0 + R * (p1 - p0);
%     p22 = p0 + R * (p2 - p0);
%     p33 = p0 + R * (p3 - p0);
%     p44 = p0 + R * (p4 - p0);
%     pp  = [p11 p22 p33 p44];
%     
%     x_low  = min(pp(1,:));
%     x_high = max(pp(1,:));
%     y_low  = min(pp(2,:));
%     y_high = max(pp(2,:));
%     
%     imagesc([x_low x_high], [y_low y_high],imrotate(marker{ii},phi))
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
for tt=1:100:maxsteps
    
    clf
    set(gcf, 'Position', position)
    hold on
    for oo = 1:length(obstacles)
        plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2)
    end
    for ii=1:1:nAgents
        plot(x(tt,ii,1), x(tt,ii,2),'o','Color',color(ii),'Linewidth',lw,'MarkerSize',mksz);
%         x_low  = x(tt,ii,1) - markersize(1)/2; % Left edge of marker
%         x_high = x(tt,ii,1) + markersize(1)/2; % Right edge of marker
%         y_low  = x(tt,ii,2) - markersize(2)/2; % Bottom edge of marker
%         y_high = x(tt,ii,2) + markersize(2)/2; % Top edge of marker
%         
%         phi = x(tt,ii,3);
%         p0  = [x(tt,ii,1); x(tt,ii,2)];
%         R   = [cos(phi) sin(phi); -sin(phi) cos(phi)];
%         p1  = [x_high; y_low];
%         p2  = [x_high; y_high];
%         p3  = [x_low; y_high];
%         p4  = [x_low; y_low];
% 
%         p11 = p0 + R * (p1 - p0);
%         p22 = p0 + R * (p2 - p0);
%         p33 = p0 + R * (p3 - p0);
%         p44 = p0 + R * (p4 - p0);
%         pp  = [p11 p22 p33 p44];
% 
%         x_low  = min(pp(1,:));
%         x_high = max(pp(1,:));
%         y_low  = min(pp(2,:));
%         y_high = max(pp(2,:));
% 
%         imagesc([x_low x_high], [y_low y_high],imrotate(marker{ii},phi))
%         imagesc([x_low x_high], [y_low y_high],marker{ii})
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