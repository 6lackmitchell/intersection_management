function cinematographer(x,obstacles,filename)


color = ['b','g','y','m'];
lw    = 3.0;
maxsteps = size(x,1);
nAgents = size(x,2);
maxXdim = 13;
maxYdim = 13;

% Make the movie object
mov = struct('cdata', [], 'colormap', []);

vidObj = VideoWriter(filename, 'MPEG-4');

% Frame rate. Keep this at 24 if you don't care.
FR = 30;
vidObj.FrameRate = FR;
open(vidObj)

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
    plot(x(1,ii,1), x(1,ii,2),'o','Color',color(ii),'Linewidth',lw);
    axis([-maxXdim maxXdim -maxYdim maxYdim]);
end
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
for tt=1:10:maxsteps
    
    clf
    set(gcf, 'Position', position)
    hold on
    for oo = 1:length(obstacles)
        plot(obstacles(oo).x,obstacles(oo).y,'Color',obstacles(oo).color,'Linewidth',lw+2)
    end
    for ii=1:1:nAgents
        plot(x(tt,ii,1), x(tt,ii,2),'o','Color',color(ii),'Linewidth',lw);
        axis([-maxXdim maxXdim -maxYdim maxYdim]);
    end
    
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