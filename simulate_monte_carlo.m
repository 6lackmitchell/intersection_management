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
% Dec 2021; Last revision: 9 -Dec-2021

% Framework Setup
clc; clear; close all; restoredefaultpath;

% Dynamics and Controller Settings
campaign       = "testing";
dyn_mode       = "dynamic_bicycle_rdrive_1u";
con_mode       = "ff_cbf";
cbf_type       = "nominal_cbf";
pmetric        = "no_priority";
cost_mode      = "costs";
im_used        = 0;
backup         = false;
pcca           = false;
input_bounds   = true;
class_k_l0     = 10.0;

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
run(strcat('dynamics/',dyn_mode,'/initial_conditions.m'))
u_params = load(strcat('./controllers/',con_mode,'/control_params.mat'));

% Monte Carlo Parameters
nTrials        = 1000;
nNon           = 1;
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
% parfor nn = 1:nTrials
for nn = 1:nTrials

    % Set up trial
    [x0_new,Tpath_new]  = randomize_ic(x0,Tpath,dyn_mode);
    t_params    = struct('nTimesteps',nTimesteps,'ti',ti,'tf',tf,'dt',dt);
    settings    = struct('x0',x0,'x0i',x0_new,'Tpath',{Tpath_new},'nAgents',nAgents,...
                         'nStates',nStates,'nControls',nControls,'xGoal',{xGoal},...
                         'Rpath',{Rpath},'path',{path},'im_used',im_used,'Lr',Lr,...
                         'SL',SL,'nNon',nNon,'pcca',pcca,'pmetric',pmetric,...
                         'cbf_type',cbf_type,'class_k_l0',class_k_l0,'input_bounds',input_bounds,...
                         'backup',backup);
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
file_settings = struct('campaign',campaign,'dyn_mode',dyn_mode,'cbf_txt',cbf_type,'pmetric',pmetric,'input_bounds',input_bounds,'backup',backup,'pcca',pcca);
file_description = get_file_description(file_settings);
filename = strcat(file_description,con_mode,'_',num2str(nAgents),'MonteCarlo_N',num2str(nTrials),'Nnon',num2str(nNon),'_K',num2str(class_k_l0),'_NNbrake.mat');
save(filename)

%% Analyze Throughput Results
% 01.13.2022
% to_load  = 'datastore/double_integrator/monte_carlo/normal_cbf/high_effort/ff_cbf_4MonteCarlo_N1000_testing'
% load(filename);

TTI     = Inf*ones(nTrials*nAgents,1);
vvios   = zeros(nTrials,1);
pvios   = zeros(nTrials,1);
infeas  = zeros(nTrials,1);
dlock   = zeros(nTrials,1);
endtime = zeros(nTrials,1);
successes = zeros(nTrials,1);
vio_mags  = zeros(nTrials,1);
for nn = 1:nTrials
    TTI((nn-1)*nAgents+1:(nn-1)*nAgents+nAgents) = trial_data(nn).TTI;
    infeas(nn)  = trial_data(nn).code == 0;
    endtime(nn) = trial_data(nn).t;
    dlock(nn)   = endtime(nn) == 20;
    successes(nn) = trial_data(nn).success;
    vvios(nn)   = sum(trial_data(nn).vios(:,:,1),'all') > 0;
    pvios(nn)   = sum(trial_data(nn).vios(:,:,2),'all') > 0;
    if pvios(nn) > 0
        vio_mags(nn) = min(trial_data(nn).vmags);
    end
end

success_rate  = sum(successes) / nTrials
successes_idx = find(successes == 1);
if isempty(successes_idx)
    average_time = 0
else
    average_time  = sum([trial_data(successes_idx).t]) / sum(successes)
end

sortedTTI  = sort(TTI);
finished   = sortedTTI(find(sortedTTI < 5.0));
unfinished = sortedTTI(find(sortedTTI >= 5.0));

fraction_finished   = length(finished) / (nAgents*nTrials);
fraction_unfinished = 1 - fraction_finished;

fraction_infeasible = sum(infeas) / nTrials;

fraction_complete   = 1 - fraction_infeasible - sum(pvios) / nTrials;
fraction_feasible   = 1 - fraction_infeasible
fraction_deadlock   = sum(dlock) / nTrials

% fraction_virt_vio   = sum(vvios) / nTrials
fraction_phys_vio   = sum(pvios) / nTrials
avg_phys_vio        = mean(vio_mags(find(vio_mags < 0)));
max_phy_vio         = max(abs(vio_mags(find(vio_mags < 0))))

mean_all            = mean(finished,'all');
mean_endtime        = mean(endtime(find(infeas==1))); 

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
x0_og      = misc_params.x0;
x0         = misc_params.x0i;
Tpath      = misc_params.Tpath;
Rpath      = misc_params.Rpath;
path       = misc_params.path;
xGoal      = misc_params.xGoal;
im_used    = misc_params.im_used;
Lr         = misc_params.Lr;
SL         = misc_params.SL;
pcca       = misc_params.pcca;
pmetric    = misc_params.pmetric;
cbf_type   = misc_params.cbf_type;
classk     = misc_params.class_k_l0;
ubounds    = misc_params.input_bounds;
backup     = misc_params.backup;

x          = zeros(nTimesteps,nAgents,nStates);
u          = zeros(nTimesteps,nAgents,nControls);
u0         = zeros(nTimesteps,nAgents,nControls);
sols       = zeros(nTimesteps,nAgents,nAgents*nControls+4);
priority   = zeros(nTimesteps,nAgents);
violations = zeros(nTimesteps,nAgents,2);
vio_mag    = zeros(nTimesteps,1);
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
success   = zeros(nAgents,1);
tol       = 0.25;

ii = 1;

% for ii = 1:nTimesteps
while ii <= nTimesteps
    t   = ii *  dt;
    xx  = squeeze(x(ii,:,:));

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
        if newidx == size(xGoal{aa},1)% - 1
            thru_time(aa) = t;
            success(aa) = 1;
        end

        if sum(success) == nAgents
            break
        end

        % Specify path segment
        old_gidx    = gidx(aa);
        gidx(aa)    = min(newidx,size(xGoal{aa},1));

        % Assign path segment data
        if old_gidx ~= gidx(aa) || ii == 1
            t0(aa)      = t;
            xS(aa,:)    = xx(aa,1:2); % Old way

            if strcmp(func2str(dynamics),'double_integrator')
                th0(aa) = atan2(xx(aa,4),xx(aa,3));
            else
                th0(aa) = xx(aa,3);
            end

        end

        % Consolidate path segment data and get trajectory info
        Tfxt(aa) = Tpath{aa}(gidx(aa));

%         % Adjust Tfxt according to priority
%         Tfxt(aa) = adjust_fxts_time(priority);

        settings = struct('T',    Tfxt(aa),             ...
                          't0',   t0(aa),               ...
                          'xS',   xS(aa,:),             ...
                          'xF',   xGoal{aa}(gidx(aa),:),...
                          'th0',  th0(aa),              ...
                          'R',    Rpath{aa}(gidx(aa)),  ...
                          'path', path{aa}{gidx(aa)});
        [r(aa,:),rdot(aa,:),r2dot(aa,:)] = trajectories(t,xx(aa,:),settings);

    end

    settings          = struct('dynamics', @dynamics, ...
                               'uLast',    uLast,     ...
                               'tSlots',   tSlots ,   ...
                               'wHat',     wHat,      ...
                               'Tfxt',     Tfxt,      ...
                               'r',        r,         ...
                               'rdot',     rdot,      ...
                               'r2dot',    r2dot,     ...
                               'Lr',       Lr,        ...
                               'SL',       SL,        ...
                               't0',       t0,        ...
                               'prior',    priority(max(ii-1,1),:), ...
                               'pcca',     pcca,      ...
                               'pmetric',  pmetric,   ...
                               'cbf_type', cbf_type,  ...
                               'classk',   classk,    ...
                               'ubounds',  ubounds,   ...
                               'backup',   backup,    ...
                               'Nn',       nNon,      ...
                               'dt',       dt);
    try
        % Compute control input
        data         = controller(t,xx,settings,u_params);

        if data.code == 1

            % Organize data
            u(ii,:,:)       = data.u;
            sols(ii,:,:)    = data.sols;
            uLast           = data.uLast;
    %         safety(ii,:)    = data.cbf;
            tSlots          = data.tSlots;
            u0(ii,:,:)      = data.uNom;
            wHat            = data.wHat;
            priority(ii,:)  = data.prior;
            violations(ii,:) = [data.v_vio; data.p_vio]';
        elseif t == dt
            [x0_new,Tpath_new]  = randomize_ic(x0_og,Tpath,func2str(dynamics));
            x(ii,:,:) = x0_new;
            Tpath = Tpath_new;
            continue
        elseif data.code == -1
            violations(ii,:) = [data.v_vio; data.p_vio]';
            vio_mag(ii)      = data.vio_mag;
            break
        elseif data.code == 3 || data.code == 4
            data.code = 0;
            break
        else
            data.code = 0;
            break
        end

%         deadlock1 = 0;%(sum(xx(:,4).^2) < 0.01);
%         deadlock2 = 0;
%         if t > 3
%             dx = squeeze(x(ii,:,1:2) - x(ii-floor(3/dt),:,1:2));
%             deadlock2 = any(sum(dx'.^2)<0.1);
%         end
% 
%         if deadlock1 || deadlock2
%             data.code = -2;
%             break
%         end

    catch ME
        break
    end

    % idx of vehicle outside 20m from intersection center
    outside_idx = find(sqrt(sum(squeeze(x(ii,:,1:2))'.^2)) > 20);
    if sum(success) == nAgents || any(success(outside_idx) == 0)
        break
    end

    % Update Dynamics
    xdot          = dynamics(t,squeeze(x(ii,:,:)),squeeze(u(ii,:,:)),settings);
    x(ii + 1,:,:) = x(ii,:,:) + dt * reshape(xdot,[1 size(xdot)]);

    % Restrict Angles to between -pi and pi
    if ~strcmp(func2str(dynamics),'double_integrator')
        x(ii + 1,:,3) = wrapToPi(x(ii + 1,:,3));
    end

    % Iterate
    ii = ii + 1;

end

trial_data = struct('success', sum(success)==nAgents, ...
                    'code',    data.code,             ...
                    'TTI',     thru_time,             ...
                    'x',       x,                     ...  
                    'u',       u,                     ...
                    'u0',      u0,                    ...
                    't',       t,                     ...
                    'sols',    sols,                  ...
                    'vios',    violations,            ...,
                    'vmags',   vio_mag);

end

function ret = data_content(nTimesteps,nAgents,nStates)
ret = struct('success', 0,                                 ...
             'code',    0,                                 ...
             'TTI',     Inf*ones(nAgents,1),               ...
             'sols',    zeros(nTimesteps,nAgents,nAgents*2+factorial(nAgents-1)),...
             'vios',    zeros(nTimesteps,nAgents,2),       ...
             'vmags',   zeros(nTimesteps,1),               ...
             'x',       zeros(nTimesteps,nAgents,nStates), ...  
             'u',       zeros(nTimesteps,nAgents,2),       ...
             'u0',      zeros(nTimesteps,nAgents,2),       ...
             't',       0);
end

function [x0_rand,Tpath] = randomize_ic(x0,Tpath,dyn_mode)
    x0_rand       = x0;
    d_variability = 5.0;
    s_variability = 3.0;
    avg_speed     = 7.5;

    for aa = 1:size(x0,1)
        % Random distance from intersection -- uniform dist.
        rand_dist  = (2*d_variability)*(rand(1) - 0.5);
        rand_speed = (2*s_variability)*(rand(1) - 0.5);

        % Randomize Distance from intersection and adjust time
        if abs(x0(aa,2)) == 1.5
            x0_rand(aa,1) = x0(aa,1) + rand_dist;
            Tpath{aa}(1)  = abs(x0_rand(aa,1) / avg_speed);
        else
            x0_rand(aa,2) = x0(aa,2) + rand_dist;
            Tpath{aa}(1)  = abs(x0_rand(aa,2) / avg_speed);
        end
    
        % Random speed
        if ~strcmp(dyn_mode,'double_integrator')
            x0_rand(aa,4) = x0(aa,4) + rand_speed;
        else
            if abs(x0(aa,2)) == 1.5
                x0_rand(aa,3) = x0(aa,3) + rand_speed;
            else
                x0_rand(aa,4) = x0(aa,4) + rand_speed;
            end
        end

    end
end

function [ret] = past_goal(xa,xg,x0)
Rcondition  = norm(xa) > norm(xg);
Acondition  = dot(xg,xa) / (norm(xg)*norm(xa)) >  cos(deg2rad(60));
X0condition = dot(xg,x0) / (norm(xg)*norm(x0)) < -cos(deg2rad(60));

ret = Rcondition && Acondition && X0condition;

end

function [file_description] = get_file_description(txt_settings)
campaign        = txt_settings.campaign;
dyn_mode        = txt_settings.dyn_mode;
cbf_txt         = txt_settings.cbf_txt;
priority_metric = txt_settings.pmetric;

input_txt = 'input_constraints';
if ~txt_settings.input_bounds
    input_txt = strcat('no_',input_txt);
end

backup_txt = 'backup';
if ~txt_settings.backup
    backup_txt = strcat('no_',backup_txt);
end

pcca_txt = 'pcca';
if ~txt_settings.pcca
    pcca_txt = strcat('no_',pcca_txt);
end

file_description = strcat('datastore/',campaign,'/',dyn_mode,'/decentralized/',backup_txt,'/',input_txt,'/',pcca_txt,'/',cbf_txt,'/',priority_metric,'/');

end
