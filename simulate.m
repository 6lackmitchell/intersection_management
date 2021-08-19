%% Framework Setup
clc; clear; close all;
%#ok<*NASGU>

% Add Desired Paths
folders = {'controllers','datastore','dynamics','settings'};
for ff = 1:length(folders)
    addpath(folders{ff})
    d = dir(folders{ff});
    isub = [d(:).isdir]; %# returns logical vector
    subFolders = {d(isub).name}';
    subFolders(ismember(subFolders,{'.','..'})) = [];
    for fff = 1:length(subFolders)
        addpath(strcat(folders{ff},'/',subFolders{fff}));
    end
end

%% Initialize Simulation Parameters

% Define Dynamics and Controller modes
dyn_mode       = "kinematic_bicycle_rdrive";
con_mode       = "queue_control";

run('settings/timing.m')
run(strcat('settings/',dyn_mode,'/initial_conditions.m'))
run(strcat('settings/',con_mode,'/control_params.m'))

% State Logging Variables
x     = zeros(nTimesteps,nAgents,nStates);
xHat  = zeros(nTimesteps,nAgents,nStates);
xErr  = zeros(nTimesteps,nAgents,nStates);

% Control Logging Variables
u        = zeros(nTimesteps,nAgents,nControls);
v        = zeros(nTimesteps,nAgents,nCBFs);
uNom     = zeros(nTimesteps,nAgents,nControls);
uLast    = zeros(nAgents,nControls);
uLast    = zeros(nAgents,nControls*nAgents);

thetaHat = zeros(nTimesteps,nAgents,nCBFs);

% Safety and Performance Logging Variables
safety      = zeros(nTimesteps,nAgents,nCBFs);
safety      = zeros(nTimesteps,nAgents);
performance = zeros(nTimesteps,nAgents,1);

% Define Controller
controller = str2func(con_mode);

% Specify initial conditions
for aa = 1:nAgents
    x(1,aa,:) = x0(aa,1:nStates);
end

% Reservation-based Objects
tSlots = inf*ones(nAgents,2);

% More Settings
quit_flags = zeros(nAgents,1);
t0         = zeros(nAgents,1);
xS         = zeros(nAgents,2);
th0        = zeros(nAgents,1);
r          = zeros(nAgents,2);
rdot       = zeros(nAgents,2);
rddot      = zeros(nAgents,2);
gidx       = ones(nAgents,1);
Tfxt       = ones(nAgents,1);

%% Execute Simulation
for ii = 1:nTimesteps
    t                 = ii *  dt;
    xx                = squeeze(x(ii,:,:));
    
    % Update Settings
    for aa = 1:nAgents
        addidx      = (norm(xGoal{aa}(gidx(aa),:) - xx(aa,1:2)) < tol && t > tSlots(aa,1));
        newidx      = gidx(aa) + addidx;
        
        if newidx > size(xGoal{aa},1)
            quit_flags(aa) = 1;
        end
        
        old_gidx    = gidx(aa);
        gidx(aa)    = min(newidx,size(xGoal{aa},1));
        
        if old_gidx ~= gidx(aa) || ii == 1
            t0(aa)      = t;
            xS(aa,:)    = xx(aa,1:2);
            th0(aa)     = xx(aa,3);
        end
        
        settings = struct('T',    Tpath{aa}(gidx(aa)),...
                          't0',   t0(aa),                   ...
                          'xS',   xS(aa,:),                   ...
                          'xF',   xGoal{aa}(gidx(aa),:),...
                          'th0',  th0(aa),                  ...
                          'R',    Rpath{aa}(gidx(aa)),   ...
                          'path', path{aa}{gidx(aa)});
        [r(aa,:),rdot(aa,:),rddot(aa,:)] = trajectories(t,xx(aa,:),settings);
        
        Tfxt(aa) = Tpath{aa}(gidx(aa));
                         
    end
    
    if all(quit_flags == 1)
        break
    end
    
    settings          = struct('dynamics', @dynamics,...
                               'uMode',    con_mode, ...
                               'uLast',    uLast,    ...
                               'tSlots',   tSlots ,  ...
                               'Gamma',    Gamma,    ...
                               'e1',       e1,       ...
                               'e2',       e2,       ...
                               'Tfxt',     Tfxt,     ...
                               'r',        r,        ...
                               'rdot',     rdot,     ...
                               'rddot',    rddot);
    try
        % Compute control input
        data         = controller(t,xx,settings);
        
        % Organize data
        u(ii,:,:)    = data.u;
        uLast        = data.uLast;
        safety(ii,:) = data.cbf;
        tSlots       = data.tSlots;
        uNom(ii,:,:) = data.uNom;
        
    catch ME
        t
        disp(ME.message)
        rethrow(ME)
        break
    end

    % Update Dynamics
    [xdot,f,g]    = dynamics(dyn_mode,t,squeeze(x(ii,:,:)),squeeze(u(ii,:,:)));
    x(ii + 1,:,:) = x(ii,:,:) + dt * reshape(xdot,[1 size(xdot)]);
    
end
beep

%% Plot Simulation Results
% filename = 'datastore/single_integrator/four_ellipses_distributed.mat';
% load(filename)
filename = strcat('datastore/',dyn_mode,'/',con_mode,'_4car_intersection.mat');
% tf = 2.5;
ii = t / dt;

lw = 3.0;
tt = linspace(dt,ii*dt,ii);
edge_SEvx =  3.0*ones(ii,1);
edge_SEvy = linspace(-13.0,-3.0,ii);
edge_SEhx = linspace(3.0,13.0,ii);
edge_SEhy = -3.0*ones(ii,1);

edge_SWvx = -3.0*ones(ii,1);
edge_SWvy = linspace(-13.0,-3.0,ii);
edge_SWhx = linspace(-13.0,-3.0,ii);
edge_SWhy = -3.0*ones(ii,1);

edge_NEvx =  3.0*ones(ii,1);
edge_NEvy = linspace(3.0,13.0,ii);
edge_NEhx = linspace(3.0,13.0,ii);
edge_NEhy =  3.0*ones(ii,1);

edge_NWvx = -3.0*ones(ii,1);
edge_NWvy = linspace(3.0,13.0,ii);
edge_NWhx = linspace(-13.0,-3.0,ii);
edge_NWhy =  3.0*ones(ii,1);

center_Sx =  0.0*ones(ii,1);
center_Sy = linspace(-13.0,-3.0,ii);

center_Nx =  0.0*ones(ii,1);
center_Ny = linspace(3.0,13.0,ii);

center_Ex = linspace(3.0,13.0,ii);
center_Ey = 0.0*ones(ii,1);

center_Wx = linspace(-13.0,-3.0,ii);
center_Wy = 0.0*ones(ii,1);

obstacles = [struct('x',edge_SEvx,'y',edge_SEvy,'color','k'),
             struct('x',edge_SEhx,'y',edge_SEhy,'color','k'),
             struct('x',edge_SWvx,'y',edge_SWvy,'color','k'),
             struct('x',edge_SWhx,'y',edge_SWhy,'color','k'),
             struct('x',edge_NEvx,'y',edge_NEvy,'color','k'),
             struct('x',edge_NEhx,'y',edge_NEhy,'color','k'),
             struct('x',edge_NWvx,'y',edge_NWvy,'color','k'),
             struct('x',edge_NWhx,'y',edge_NWhy,'color','k'),
             struct('x',center_Sx,'y',center_Sy,'color','y'),
             struct('x',center_Nx,'y',center_Ny,'color','y'),
             struct('x',center_Ex,'y',center_Ey,'color','y'),
             struct('x',center_Wx,'y',center_Wy,'color','y')];



% load(road_objects.mat)
% ell1x = cx1 + dx1.*cos(2*pi*tt); ell1y = cy1 + dy1.*sin(2*pi*tt);
% ell2x = cx2 + dx2.*cos(2*pi*tt); ell2y = cy2 + dy2.*sin(2*pi*tt);
% ell3x = cx3 + dx3.*cos(2*pi*tt); ell3y = cy3 + dy3.*sin(2*pi*tt);
% ell4x = cx4 + dx4.*cos(2*pi*tt); ell4y = cy4 + dy4.*sin(2*pi*tt);

% figure(1);
% title('State Trajectories')
% hold on
% plot(ell1x,ell1y,'k','LineWidth',lw)
% plot(ell2x,ell2y,'k','LineWidth',lw)
% plot(ell3x,ell3y,'k','LineWidth',lw)
% plot(ell4x,ell4y,'k','LineWidth',lw)
% for jj = 3:nAgents
%     plot(tt,x(1:ii,jj,1),'LineWidth',lw)
% end
% hold off

figure(2);
title('Control Inputs X')
hold on
for jj = 1:nAgents
    plot(tt,u(1:ii,jj,1),'LineWidth',lw)
    plot(tt,uNom(1:ii,jj,1),'LineWidth',lw)
end
legend('\omega_1','\omega_{1,nom}','\omega_2','\omega_{2,nom}','\omega_3','\omega_{3,nom}')
hold off

figure(3);
title('Control Inputs Y')
hold on
for jj = 1:nAgents
    plot(tt,u(1:ii,jj,2),'LineWidth',lw)
    plot(tt,uNom(1:ii,jj,2),'LineWidth',lw)

%     plot(tt,u_nom(1:ii,jj,1),'LineWidth',lw)
end
legend('a_1','a_{1,nom}','a_2','a_{2,nom}','a_3','a_{3,nom}')
hold off

figure(4);
title('CBFs')
hold on
for jj = 1:nAgents
    plot(tt,safety(1:ii,jj),'LineWidth',lw)
end
hold off
% 
% figure(3);
% title('CLFs')
% hold on
% for jj = 1:nAgents
%     plot(performance(1:ii,jj,1),'LineWidth',lw)
% end
% hold off

%% Save Simulation Results
save(filename)

%% Create Movie
% obstacles = [struct('x',ell1x,'y',ell1y),struct('x',ell2x,'y',ell2y),struct('x',ell3x,'y',ell3y),struct('x',ell4x,'y',ell4y)];
moviename = erase(filename,'.mat');
cinematographer(dt,x(1:(ii),:,:),obstacles,moviename)