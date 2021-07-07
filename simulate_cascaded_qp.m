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
xGoal = x_goal(:,1:nStates);

% Control Logging Variables
u        = zeros(nTimesteps,nAgents,nControls);
v        = zeros(nTimesteps,nAgents,nCBFs);
v_track  = zeros(nTimesteps,nAgents,2);
thetaHat = zeros(nTimesteps,nAgents,nCBFs);

% Safety and Performance Logging Variables
safety      = zeros(nTimesteps,nAgents,nCBFs);
performance = zeros(nTimesteps,nAgents,1);

% Define Dynamics and Controller modes
sint_dyn       = "single_integrator";
dint_dyn       = "double_integrator";
dyn_mode       = dint_dyn;
% dyn_mode       = sint_dyn;
con_mode       = "centralized";
con_mode       = "distributed";

% Define Controllers
controller         = @clf_cbf_qp;
controller         = @centralized_cbf_qp;
controller         = @distributed_cbf_qp;
nominal_controller = @hl_fxt_clf_qp;

% Define Cost Functional
cost               = @min_diff_nominal;

% Specify initial conditions
for aa = 1:nAgents
    x(1,aa,:) = x0(aa,1:nStates);
end

%% Execute Simulation
for ii = 1:nTimesteps
    t                 = ii *  dt;
    
    % Decentralized Controller
    if con_mode == "distributed"
        
        for aa = 1:nAgents
            
            % Configure variables
            xx         = squeeze(x(ii,:,:));
            xg         = xGoal(aa,1:2);
            xo         = squeeze(x(ii,idx(idx ~= aa),:));
            
%             % Configure agent variables
%             xx          = squeeze(x(ii,aa,:))';
%             xo          = squeeze(x(ii,idx(idx ~= aa),:));
%             xg          = xGoal(aa,:);
            
            % Get single integrator dynamics
            [drift,f,g] = dynamics(sint_dyn,t,xx(aa,1:2),[0]);
            s_dyn         = struct('fg',{f,g});
            [drift,f,g] = dynamics(dint_dyn,t,xx,[0]);
            d_dyn         = struct('fg',{f,g});

            % Compute Nominal and Safe Control Inputs
            v_track(ii,aa,:)        = nominal_controller(t,xx,xg,aa,cost,s_dyn);        
            [u(ii,aa,:),v(ii,aa,:)] = controller(t,xx(aa,:),xo,squeeze(v_track(ii,aa,:)),cost,d_dyn);

            % Log Safety and Performance
%             safety(ii,aa,:)         = B(t,xx,xo);
            performance(ii,aa,:)    = V(t,xx(aa,1:2)-xg,x,aa);

        end
    
    % Centralized Controller
    elseif con_mode == "centralized"
        % Configure arguments to controller functions
        xx          = squeeze(x(ii,:,:));
        xx_flat     = reshape(squeeze(x(ii,:,:)).',1,[]);
        xg          = reshape(xGoal.',1,[]);
        [drift,f,g] = dynamics(dyn_mode,t,xx,[0]);
        dyn         = struct('fg',{f,g});
        
        % Compute nominal and safe control inputs
        u_nom_flat            = nominal_controller(t,xx_flat,xg,cost,dyn);
        u_nom(ii,:,:)         = permute(reshape(u_nom_flat,[1 nControls nAgents]),[1 3 2]);
        [u_flat,v_flat]       = controller(t,xx,u_nom_flat,cost,dyn);
        u(ii,:,:)             = permute(reshape(u_flat,[1 nControls nAgents]),[1 3 2]);
        v(ii,:,:)             = permute(reshape(v_flat,[1 nCBFs nAgents]),[1 3 2]);
        
        % Log Performance and Safety Constraints
        performance(ii,1,:)   = V(t,xx_flat-xg);

   
    end

    % Update Dynamics
    [xdot,f,g]    = dynamics(dyn_mode,t,squeeze(x(ii,:,:)),squeeze(u(ii,:,:)));
    x(ii + 1,:,:) = x(ii,:,:) + dt * reshape(xdot,[1 size(xdot)]);
    
end
beep

%% Plot Simulation Results
% filename = 'datastore/single_integrator/four_ellipses_distributed.mat';
% load(filename)
lw = 3.0;
tt = linspace(dt,t,nTimesteps);
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
    plot(tt,v_track(:,ii,1),'LineWidth',lw)
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
    plot(tt,performance(:,ii,1),'LineWidth',lw)
end
hold off

%% Save Simulation Results
filename = strcat('datastore/',dyn_mode,'/',con_mode,'_four_ellipses_TEST2.mat');
save(filename)

%% Create Movie
obstacles = [struct('x',ell1x,'y',ell1y),struct('x',ell2x,'y',ell2y),struct('x',ell3x,'y',ell3y),struct('x',ell4x,'y',ell4y)];
moviename = erase(filename,'.mat');
cinematographer(x,obstacles,moviename)