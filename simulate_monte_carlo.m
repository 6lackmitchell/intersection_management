%simulate_monte_carlo - Framework for performing Monte Carlo simulations
%for dynamical systems. This script will use a variety of initial
%conditions to simulate the trajectories of the controlled dynamical system
%of interest. MATLAB's parallel computing capabilities are utilized in the
%form of the parfor loop.
%
% Syntax:  simulate_monte_carlo
%
% Other m-files required: simulate
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% Dec 2021; Last revision: 9-Dec-2021

% Framework Setup
clc; clear; close all; restoredefaultpath;

% Dynamics and Controller modes
mode           = "Centralized Priority-Cost Allocation";
% dyn_mode       = "dynamic_bicycle_rdrive";
dyn_mode       = "dynamic_bicycle_rdrive_1u";
con_mode       = "ff_cbf_cpca_rails";
cost_mode      = "costs";
im_used        = 0;

% Add Desired Paths
% addpath '/Library/gurobi912/mac64/matlab'; % For mac
addpath 'C:\gurobi950\win64\matlab'; % For Thinkstation
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

% Load settings into workspace
run('settings/timing.m')
run(strcat('dynamics/',dyn_mode,'/initial_conditions_cpca.m'))
u_params = load(strcat('./controllers/',con_mode,'/control_params.mat'));

% Monte Carlo Parameters
nTrials        = 1000;
nNon           = 0;
trial_data     = repmat(data_content(nTimesteps,nAgents,nStates),nTrials,1);
time_through_intersection = zeros(nTrials,nAgents);

% Logging Variables
x = zeros(nTrials,nTimesteps,nAgents,nStates);
u = zeros(nTrials,nTimesteps,nAgents,nControls);

% Safety and Performance Logging Variables
safety = zeros(nTrials,nTimesteps,nAgents);

% Define Dynamics and Controller
dynamics   = str2func(dyn_mode);
controller = str2func(con_mode);

%% Execute Monte Carlo Simulation
tic
parfor nn = 1:nTrials

    % Set up trial
    [x0_new,Tpath_new]  = randomize_ic(x0,Tpath);
    t_params    = struct('nTimesteps',nTimesteps,'ti',ti,'tf',tf,'dt',dt);
    settings    = struct('x0',x0_new,'Tpath',{Tpath_new},'nAgents',nAgents,'nStates',nStates,'nControls',nControls,'xGoal',{xGoal},'Rpath',{Rpath},'path',{path},'im_used',im_used,'Lr',Lr,'SL',SL,'nNon',nNon);
    trial_setup = struct('dynamics',   dynamics,   ...
                         'controller', controller, ...
                         'u_params',   u_params,   ...
                         't_params',   t_params,   ...
                         'settings',   settings);

    trial_data(nn) = run_one_trial(trial_setup);

    % Simulation Progress
    if mod(nn,nTrials/10) == 0
        disp("************  Percent Completed: "+num2str(fix(nn/nTrials*10^2))+"%  ************")
    end

end
toc
beep

%% Save Simulation Results
filename = strcat('datastore/',dyn_mode,'/monte_carlo/fcfs_speed/',con_mode,'_',num2str(nAgents),'MonteCarlo',num2str(nTrials),'_intersection_tests.mat');
save(filename)

% to_load  = 'datastore/dynamic_bicycle_rdrive/monte_carlo/ff_cbf_cpca_4MonteCarlo1000_baseline_staticpriority_for.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive/monte_carlo/without_past/ff_cbf_cpca_4MonteCarlo1000_wealthredistribution_parfor.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive/monte_carlo/ff_cbf_cpca_4MonteCarlo1000_richgetricher_parfor.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive/monte_carlo/ff_cbf_cpca_4MonteCarlo1000_lowvel_highpriority.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive/monte_carlo/ff_cbf_cpca_4MonteCarlo1000_highvel_highpriority.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive/monte_carlo/ff_cbf_cpca_4MonteCarlo1000_lowDfromCenter_highpriority.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive/monte_carlo/ff_cbf_cpca_4MonteCarlo1000_highDfromCenter_highpriority.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive/monte_carlo/without_past/ff_cbf_cpca_4MonteCarlo1000_highDfromCenter_highpriority.mat'

% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_baseline_staticpriority.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_lowdev_highpri.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_highdev_highpri.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_highdev_highpri_alpha5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_4MonteCarlo1000_lowvel_highpriority.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_4MonteCarlo1000_highvel_highpriority.mat'





% %% Plot Simulation Results
% ii = fix(t / dt);
% tt = linspace(dt,ii*dt,ii);
% filename = strcat('datastore/',dyn_mode,'/',con_mode,'_',num2str(nAgents),'intersection_tests.mat');



%% Analyze Throughput Results
% 01.13.2022
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_highdev_highpri_T0p5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_highdev_highpri_T1.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_highdev_highpri.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_highdev_highpri_T1p5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_highdev_highpri_T1p5_redo.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_highdev_highpri_T2.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_highdev_highpri_T3.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_highdev_highpri_T5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_intersection_tests.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/ff_cbf_cpca_rails_4MonteCarlo1000_deviation_lowdev_highpri.mat'

% Alpha Tests
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a1.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a1p5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a2.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a2p5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a2p75.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a3.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a3p25.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a3p5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a4.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_alpha_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a10.mat'

% Lookahead Tests
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_T0p1.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a1p5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a2.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a2p5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a2p75.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a3.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a3p25.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a3p5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a4.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a5.mat'
% to_load  = 'datastore/dynamic_bicycle_rdrive_1u/monte_carlo/fully_centralized_lookahead_tests/ff_cbf_cpca_rails_4MonteCarlo1000_centralized_a10.mat'
% % 
% load(to_load);

TTI     = Inf*ones(nTrials*nAgents,1);
infeas  = zeros(nTrials,1);
endtime = zeros(nTrials,1);
for nn = 1:nTrials
    TTI((nn-1)*nAgents+1:(nn-1)*nAgents+nAgents) = trial_data(nn).TTI;
    infeas(nn)  = trial_data(nn).t < 5;
    endtime(nn) = trial_data(nn).t;
end

sortedTTI  = sort(TTI);
finished   = sortedTTI(find(sortedTTI < 5.0));
unfinished = sortedTTI(find(sortedTTI >= 5.0));

fraction_finished   = length(finished) / (nAgents*nTrials)
fraction_unfinished = 1 - fraction_finished;

fraction_infeasible = sum(infeas) / nTrials;
fraction_feasible   = 1 - fraction_infeasible

mean_all = mean(finished,'all')
mean_endtime = mean(endtime(find(infeas==1))) % Mean endtime of infeasible sims

%% Miscellaneous Helper Functions
function trial_data = run_one_trial(trial_setup)
dynamics    = trial_setup.dynamics;
controller  = trial_setup.controller;
t_params    = trial_setup.t_params;
u_params    = trial_setup.u_params;
misc_params = trial_setup.settings;

nTimesteps = t_params.nTimesteps;
dt         = t_params.dt;
tf         = t_params.tf;
nAgents    = misc_params.nAgents;
nStates    = misc_params.nStates;
nControls  = misc_params.nControls;
nNon       = misc_params.nNon;
x0         = misc_params.x0;
Tpath      = misc_params.Tpath;
Rpath      = misc_params.Rpath;
path       = misc_params.path;
xGoal      = misc_params.xGoal;
im_used    = misc_params.im_used;
Lr         = misc_params.Lr;
SL         = misc_params.SL;

x          = zeros(nTimesteps,nAgents,nStates);
u          = zeros(nTimesteps,nAgents,nControls);
priority   = zeros(nTimesteps,nAgents);
x(1,:,:)   = x0;

% More Settings
uLast     = zeros(nAgents,nControls);
tSlots    = inf*ones(nAgents,2);
t0        = zeros(nAgents,1);
xS        = zeros(nAgents,2);
th0       = zeros(nAgents,1);
r         = zeros(nAgents,2);
rdot      = zeros(nAgents,2);
r2dot     = zeros(nAgents,2);
gidx      = ones(nAgents,1);
Tfxt      = ones(nAgents,1);
wHat      = zeros(nAgents,nAgents*nControls);
thru_time = Inf*ones(nAgents,1);
success   = 1;
tol       = 0.5;


for ii = 1:nTimesteps
    t                 = ii *  dt;
    xx                = squeeze(x(ii,:,:));

%     % Simulation Progress
%     if mod(ii,1/dt) == 0
%         disp("************  Percent Completed: "+num2str(fix(t/tf*10^4)/10^2)+"%  ************")
%         disp("************  Time Elapsed:      "+num2str(fix(t*10^2)/10^2)+"s  ************")
%     end

    % Update Settings
    for aa = 1:nAgents

        % Determine which path segment is active
        reached     = norm(xGoal{aa}(gidx(aa),:) - xx(aa,1:2)) < tol;
        closer      = norm(xGoal{aa}(min(gidx(aa)+1,size(xGoal{aa},1)),:) - xx(aa,1:2)) < norm(xGoal{aa}(gidx(aa),:) - xx(aa,1:2));
        past        = past_goal(xx(aa,1:2),xGoal{aa}(gidx(aa),:),x0(aa,1:2));
        allowed     = 1*(~im_used) + im_used*(t > tSlots(aa,1));
        addidx      = (reached || closer || past) && allowed;
        newidx      = gidx(aa) + addidx;
%         if addidx == 1
%             disp("*** Agent "+num2str(aa)+" has reached Goal "+num2str(newidx-1)+"!!")
%         end

        % Set quit flag to true if final goal met
        if newidx == size(xGoal{aa},1) - 1
            thru_time(aa) = t;
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

        % Adjust Tfxt according to priority
        if ii > 1
            [~,pIdx] = sort(priority(max(ii-1,1),1:(nAgents-nNon)),'descend');
            if ismember(aa,1:(nAgents-nNon))
                if gidx(aa) == 1
                    Tfxt(aa) = Tfxt(aa) + (pIdx(aa) - 1);
                end
            end
        end

        settings = struct('T',    Tfxt(aa),             ...
                          't0',   t0(aa),               ...
                          'xS',   xS(aa,:),             ...
                          'xF',   xGoal{aa}(gidx(aa),:),...
                          'th0',  th0(aa),              ...
                          'R',    Rpath{aa}(gidx(aa)),  ...
                          'path', path{aa}{gidx(aa)});
        [r(aa,:),rdot(aa,:),r2dot(aa,:)] = trajectories(t,xx(aa,:),settings);

    end

    settings          = struct('dynamics', @dynamics,...
                               'uLast',    uLast,    ...
                               'tSlots',   tSlots ,  ...
                               'wHat',     wHat,     ...
                               'Tfxt',     Tfxt,     ...
                               'r',        r,        ...
                               'rdot',     rdot,     ...
                               'r2dot',    r2dot,    ...
                               'Lr',       Lr,       ...
                               'SL',       SL,       ...
                               't0',       t0,       ...
                               'prior',    priority(max(ii-1,1),:), ...
                               'Nn',       nNon);
    try
        % Compute control input
        data         = controller(t,xx,settings,u_params);

        % Organize data
        u(ii,:,:)       = data.u;
        uLast           = data.uLast;
%         safety(ii,:)    = data.cbf;
        tSlots          = data.tSlots;
%         uNom(ii,:,:)    = data.uNom;
        wHat            = data.wHat;
        priority(ii,:)  = data.prior;

    catch ME
%         disp(ME)
        success = 0;
        break
    end

    % Update Dynamics
    xdot          = dynamics(t,squeeze(x(ii,:,:)),squeeze(u(ii,:,:)),settings);
    x(ii + 1,:,:) = x(ii,:,:) + dt * reshape(xdot,[1 size(xdot)]);

    % Restrict Angles to between -pi and pi
    x(ii + 1,:,3) = wrapToPi(x(ii + 1,:,3));

end

trial_data = struct('success', success,   ...
                    'TTI',     thru_time, ...
                    'x',       x,         ...  
                    'u',       u,         ...
                    't',       t);

end

function ret = data_content(nTimesteps,nAgents,nStates)
ret = struct('success', 0,                                 ...
             'TTI',     Inf*ones(nAgents,1),               ...
             'x',       zeros(nTimesteps,nAgents,nStates), ...  
             'u',       zeros(nTimesteps,nAgents,2),       ...
             't',       0);
end

function [x0_rand,Tpath] = randomize_ic(x0,Tpath)
    x0_rand       = x0;
    d_variability = 0.0;
    s_variability = 4.0;
    avg_speed     = 6.0;

    for aa = 1:size(x0,1)
        % Random distance from intersection -- uniform dist.
        rand_dist  = (2*d_variability)*(rand(1) - 0.5);
        rand_speed = (2*s_variability)*(rand(1) - 0.5);

        % Randomize Distance from intersection and adjust time
        if abs(x0(aa,1)) == 1.5
            x0_rand(aa,1) = x0(aa,1) + rand_dist;
            Tpath{aa}(1)  = abs(x0_rand(aa,1) / avg_speed);
        else
            x0_rand(aa,2) = x0(aa,2) + rand_dist;
            Tpath{aa}(1)  = abs(x0_rand(aa,2) / avg_speed);
        end
    
        % Random speed
        x0_rand(aa,4) = x0(aa,4) + rand_speed;

    end
end

function [ret] = past_goal(xa,xg,x0)
Rcondition  = norm(xa) > norm(xg);
Acondition  = dot(xg,xa) / (norm(xg)*norm(xa)) >  cos(deg2rad(60));
X0condition = dot(xg,x0) / (norm(xg)*norm(x0)) < -cos(deg2rad(60));

ret = Rcondition && Acondition && X0condition;

end