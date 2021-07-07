%% Framework Setup
clc; clear; close all;
%#ok<*NASGU>

% Add Desired Paths
folders = {'controllers','dynamics','settings'};
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
run('settings/timing.m')
run('settings/environment.m')
run('controllers/setup.m')

% State Logging Variables
x     = zeros(nTimesteps,nAgents,nStates);
xHat  = zeros(nTimesteps,nAgents,nStates);
xErr  = zeros(nTimesteps,nAgents,nStates);

% Control Logging Variables
u        = zeros(nTimesteps,nAgents,nControls);
v        = zeros(nTimesteps,nAgents,nCBFs);
u_nom    = zeros(nTimesteps,nAgents,nControls);
thetaHat = zeros(nTimesteps,nAgents,nCBFs);

% Safety and Performance Logging Variables
safety      = zeros(nTimesteps,nAgents,nCBFs);
performance = zeros(nTimesteps,nAgents,1);

% Define Controller
controller = @centralized_pcca;
controller = @distributed_pcca;

% Define Dynamics and Controller modes
dyn_mode       = "single_integrator";
% dyn_mode       = "double_integrator";
con_mode       = "centralized";
con_mode       = "distributed";

% Specify initial conditions
for aa = 1:nAgents
    x(1,aa,:) = x0(aa,1:nStates);
end

%% Execute Simulation
for ii = 1:nTimesteps
    t                 = ii *  dt;
    xx                = squeeze(x(ii,:,:));
    
    % Update Settings
    for aa = 1:size(xGoal,1)
        addidx      = (norm(squeeze(xGoal(aa,gidx(aa),:) - x(ii,aa,:))) < tol);
        newidx      = gidx(aa) + addidx;
        gidx(aa)    = min(newidx,size(xGoal,2));
        xg(aa,:)    = xGoal(aa,gidx(aa),:);
        uLast(aa,:) = squeeze(u(max(ii-1,1),aa,:));
        if true%addidx > 0
            uLast = 0*uLast;
        end
    end
    
    settings          = struct('dynamics',@dynamics,'xGoal',xg,'uLast',uLast);
    
    % Compute control input
    [u(ii,:,:),data]  = controller(t,xx,settings);

    % Update Dynamics
    [xdot,f,g]    = dynamics(dyn_mode,t,squeeze(x(ii,:,:)),squeeze(u(ii,:,:)));
    x(ii + 1,:,:) = x(ii,:,:) + dt * reshape(xdot,[1 size(xdot)]);
    
end
beep

%% Plot Simulation Results
% filename = 'datastore/single_integrator/four_ellipses_distributed.mat';
% load(filename)
lw = 3.0;
tt = linspace(dt,nTimesteps*dt,nTimesteps);
ell1x = cx1 + dx1.*cos(2*pi*tt); ell1y = cy1 + dy1.*sin(2*pi*tt);
ell2x = cx2 + dx2.*cos(2*pi*tt); ell2y = cy2 + dy2.*sin(2*pi*tt);
ell3x = cx3 + dx3.*cos(2*pi*tt); ell3y = cy3 + dy3.*sin(2*pi*tt);
ell4x = cx4 + dx4.*cos(2*pi*tt); ell4y = cy4 + dy4.*sin(2*pi*tt);

figure(1);
title('State Trajectories')
hold on
plot(ell1x,ell1y,'k','LineWidth',lw)
plot(ell2x,ell2y,'k','LineWidth',lw)
plot(ell3x,ell3y,'k','LineWidth',lw)
plot(ell4x,ell4y,'k','LineWidth',lw)
for ii = 1:nAgents
    plot(x(:,ii,1),x(:,ii,2),'LineWidth',lw)
end
hold off

figure(2);
title('Control Inputs')
hold on
for ii = 1:1%nAgents
    plot(tt,u(:,ii,1),'LineWidth',lw)
    plot(tt,u_nom(:,ii,1),'LineWidth',lw)
end
legend('A1','A1_{nom}','A2','A2_nom')
hold off

% figure(2);
% title('CBFs')
% hold on
% for ii = 1:nAgents
%     plot(tt,safety(:,ii,1),'LineWidth',lw)
%     plot(tt,safety(:,ii,2),'LineWidth',lw)
% end
% hold off
% 
figure(3);
title('CLFs')
hold on
for ii = 1:nAgents
    plot(performance(:,ii,1),'LineWidth',lw)
end
hold off

%% Save Simulation Results
filename = strcat('datastore/',dyn_mode,'/',con_mode,'_four_ellipses_TEST2.mat');
save(filename)

%% Create Movie
obstacles = [struct('x',ell1x,'y',ell1y),struct('x',ell2x,'y',ell2y),struct('x',ell3x,'y',ell3y),struct('x',ell4x,'y',ell4y)];
moviename = erase(filename,'.mat');
cinematographer(x,obstacles,moviename)