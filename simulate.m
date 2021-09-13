%% Framework Setup
clc; clear; close all; restoredefaultpath;

% Define Dynamics and Controller modes
dyn_mode       = "dynamic_bicycle_rdrive";
con_mode       = "pcca";
cost_mode      = "costs";

% Add Desired Paths
folders = {'controllers','datastore','dynamics','helpers','settings'};
for ff = 1:length(folders)
    addpath(folders{ff})
    d = dir(folders{ff});
    isub = [d(:).isdir]; %# returns logical vector
    subFolders = {d(isub).name}';
    subFolders(ismember(subFolders,{'.','..'})) = [];
    for fff = 1:length(subFolders)
        if strcmp(subFolders{fff},dyn_mode) || strcmp(subFolders{fff},con_mode) || strcmp(subFolders{fff},cost_mode)
            addpath(strcat(folders{ff},'/',subFolders{fff}));
            d = dir(subFolders{fff});
            isubsub = [d(:).isdir]; %# returns logical vector
            subsubFolders = {d(isubsub).name}';
            subsubFolders(ismember(subsubFolders,{'.','..'})) = [];
            for ffff = 1:length(subsubFolders)
                addpath(strcat(folders{ff},'/',subFolders{fff},'/',subsubFolders{ffff}));
            end
        end
    end
end

%% Initialize Simulation Parameters

% Load settings into workspace
run('settings/timing.m')
run(strcat('dynamics/',dyn_mode,'/initial_conditions.m'))
run(strcat('controllers/',con_mode,'/control_params.m'))

% State Logging Variables
x     = zeros(nTimesteps,nAgents,nStates);
xHat  = zeros(nTimesteps,nAgents,nStates);
xErr  = zeros(nTimesteps,nAgents,nStates);

% Control Logging Variables
u        = zeros(nTimesteps,nAgents,nControls);
uNom     = zeros(nTimesteps,nAgents,nControls);
uLast    = zeros(nAgents,nControls);

% Safety and Performance Logging Variables
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
xS         = zeros(nAgents,2); % old
th0        = zeros(nAgents,1);
r          = zeros(nAgents,2);
rdot       = zeros(nAgents,2);
rddot      = zeros(nAgents,2);
gidx       = ones(nAgents,1);
Tfxt       = ones(nAgents,1);
wHat       = zeros(nAgents,nAgents*nControls);

%% Execute Simulation
for ii = 1:nTimesteps
    t                 = ii *  dt;
    xx                = squeeze(x(ii,:,:));

    % Simulation Progress
    if mod(ii,1/dt) == 0
        disp("************  Percent Completed: "+num2str(fix(t/tf*10^4)/10^2)+"%  ************")
        disp("************  Time Elapsed:      "+num2str(fix(t*10^2)/10^2)+"s  ************")
    end

    % Update Settings
    for aa = 1:nAgents

        % Determine which path segment is active
        reached     = norm(xGoal{aa}(gidx(aa),:) - xx(aa,1:2)) < tol;
        closer      = norm(xGoal{aa}(min(gidx(aa)+1,size(xGoal{aa},1)),:) - xx(aa,1:2)) < norm(xGoal{aa}(gidx(aa),:) - xx(aa,1:2));
        addidx      = (reached || closer) && t > tSlots(aa,1);
        newidx      = gidx(aa) + addidx;
        if addidx == 1
            disp("*** Agent "+num2str(aa)+" has reached Goal "+num2str(newidx-1)+"!!")
        end

        % Set quit flag to true if final goal met
        if newidx > size(xGoal{aa},1)
            quit_flags(aa) = 1;
        end

        % Specify path segment
        old_gidx    = gidx(aa);
        gidx(aa)    = min(newidx,size(xGoal{aa},1));

        % Assign path segment data
        if old_gidx ~= gidx(aa) || ii == 1
            t0(aa)      = t;
            xS(aa,:)    = xx(aa,1:2); % Old way
            th0(aa)     = xx(aa,3);
        end

        % Consolidate path segment data and get trajectory info
        Tfxt(aa) = Tpath{aa}(gidx(aa));
        settings = struct('T',    Tfxt(aa),             ...
                          't0',   t0(aa),               ...
                          'xS',   xS(aa,:),             ...
                          'xF',   xGoal{aa}(gidx(aa),:),...
                          'th0',  th0(aa),              ...
                          'R',    Rpath{aa}(gidx(aa)),  ...
                          'path', path{aa}{gidx(aa)});
        [r(aa,:),rdot(aa,:),rddot(aa,:)] = trajectories(t,xx(aa,:),settings);

    end

    % Exit simulation if all goals met
    if all(quit_flags == 1)
        break
    end

    settings          = struct('dynamics', @dynamics,...
                               'uMode',    con_mode, ...
                               'uLast',    uLast,    ...
                               'tSlots',   tSlots ,  ...
                               'wHat',     wHat,     ...
                               'Gamma',    Gamma,    ...
                               'e1',       e1,       ...
                               'e2',       e2,       ...
                               'Tfxt',     Tfxt,     ...
                               'r',        r,        ...
                               'rdot',     rdot,     ...
                               'rddot',    rddot,    ...
                               't0',       t0);
    try
        % Compute control input
        data         = controller(t,xx,settings);

        % Organize data
        u(ii,:,:)    = data.u;
        uLast        = data.uLast;
        safety(ii,:) = data.cbf;
        tSlots       = data.tSlots;
        uNom(ii,:,:) = data.uNom;
        wHat         = data.wHat;

    catch ME
        t
        disp(ME.message)
        rethrow(ME)
        break
    end

    % Update Dynamics
    [xdot,f,g]    = dynamics(dyn_mode,t,squeeze(x(ii,:,:)),squeeze(u(ii,:,:)));
    x(ii + 1,:,:) = x(ii,:,:) + dt * reshape(xdot,[1 size(xdot)]);

    % Restrict Angles to between -pi and pi
    x(ii + 1,:,3) = wrapToPi(x(ii + 1,:,3));

end
beep

%% Plot Simulation Results
ii = fix(t / dt);
tt = linspace(dt,ii*dt,ii);
filename = strcat('datastore/',dyn_mode,'/',con_mode,'_',num2str(nAgents),'car_intersection.mat');
% filename = strcat('datastore/',dyn_mode,'/',con_mode,'_',num2str(nAgents),'pcca_standardcbf_4car_intersection.mat');
% filename = strcat('datastore/',dyn_mode,'/',con_mode,'_',num2str(nAgents),'pcca_pcbf_4car_intersection.mat');

figure(2);
title('Control Inputs X')
hold on
for jj = 1:nAgents
    plot(tt,u(1:ii,jj,1),'LineWidth',lw)
%     plot(tt,atan(uNom(1:ii,jj,1)),'LineWidth',lw)
end
legend('\omega_1','\omega_2','\omega_3','\omega_4','\omega_5','\omega_6')
hold off

figure(3);
title('Control Inputs Y')
hold on
for jj = 1:nAgents
    plot(tt,u(1:ii,jj,2),'LineWidth',lw)
%     plot(tt,uNom(1:ii,jj,2),'LineWidth',lw)
end
legend('a_1','a_2','a_3','a_4','a_5','a_6')
hold off

% Load road geometry
road_file = 'datastore/geometry/road_markings.mat';       
load(road_file)

moviename = erase(filename,'.mat');
cinematographer(dt,x(1:(ii),:,:),obstacles,moviename)

%% Save Simulation Results
save(filename)