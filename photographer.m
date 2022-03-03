% Load road geometry
road_file = 'datastore/geometry/road_markings.mat';       
load(road_file)

dyn_mode = 'dynamic_bicycle_rdrive_1u';
photo(x,dyn_mode,obstacles)

function photo(x,dyn_mode,obstacles)



color = repmat(['b','k','g','r','m','c'],1,5);
lw    = 3.0;
nAgents = size(x,2);
maxXdim = 25;
maxYdim = 25;

% Plotting params
theta = 0:2*pi/101:2*pi;
RR    = 0.6;
RR    = 1.8; % This corresponds to a physical radius of 1 = RR/2

% Physical Params
L = 0.85;
L = 0.0;
% L = 1.0;

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
    if strcmp(dyn_mode,'double_integrator')
        th = atan2(x(ii,4),x(ii,3));
        cx1 = x(ii,1);
        cy1 = x(ii,2);
        cx2 = x(ii,1);
        cy2 = x(ii,2);
    else
        cx1 = x(ii,1) + L/2*cos(x(ii,3));
        cy1 = x(ii,2) + L/2*sin(x(ii,3));
        cx2 = x(ii,1) - L/2*cos(x(ii,3));
        cy2 = x(ii,2) - L/2*sin(x(ii,3));
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
    
    plot(ox1, oy1,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);
    plot(ox2, oy2,'Color',color(ii),'Linewidth',lw)%,'MarkerSize',mksz);

end
txt = strcat('t = ',num2str(0.0),' sec');
text(10,10,txt,'FontSize',14)
axis([-maxXdim maxXdim -maxYdim maxYdim]);
% hold off
drawnow update

end