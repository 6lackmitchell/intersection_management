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

% Loads a variety of settings into workspace
run('settings/simulation_setup.m') % Edit this file to change simulation setup

%% Initialize Simulation Parameters

% Monte Carlo Parameters
nTrials        = 2;
nNon           = 0;
trial_data     = repmat(data_content(nTimesteps,nAgents,nStates),nTrials,1);
time_through_intersection = zeros(nTrials,nAgents);

% Logging Variables
x = zeros(nTrials,nTimesteps,nAgents,nStates);
u = zeros(nTrials,nTimesteps,nAgents,nControls);

% Safety and Performance Logging Variables
safety = zeros(nTrials,nTimesteps,nAgents);

%% Execute Monte Carlo Simulation
tic
% parfor nn = 1:nTrials
for nn = 1:nTrials

    % Randomize initial conditions
    [x0_new,Tpath_new]  = randomize_ic(x0,Tpath,dyn_mode);

    % Formulate timing structure
    t_params             = struct();
    t_params.nTimesteps  = nTimesteps;
    t_params.ti          = ti;
    t_params.tf          = tf;
    t_params.dt          = dt;

    % Formulate misc settings structure
    settings             = struct();
    settings.x0          = x0;
    settings.x0i         = x0_new;
    settings.nControls   = nControls;
    settings.im_used     = im_used;
    settings.Lr          = Lr;
    settings.SL          = SL;
    settings.nNon        = nNon;
    settings.pcca        = pcca;
    settings.pmetric     = pmetric;
    settings.cbf_type    = cbf_type;
    settings.classK_l0   = classK_l0;
    settings.u_bounds    = input_bounds;
    settings.backup      = backup;
    settings.ppower      = ppower;

    % Formulate trial setup structure
    trial_setup          = struct();
    trial_setup.u_params = u_params;
    trial_setup.t_params = t_params;
    trial_setup.settings = settings;

    % Run one intersection scenario
    trial_data(nn)       = run_one_trial(dynamics,nominal_u,controller,trial_setup);

    % Simulation Progress
    if mod(nn,nTrials/10) == 0
        disp("************  Percent Completed: "+num2str(fix(nn/nTrials*10^2))+"%  ************")
    end

end
toc
beep

%% Save Simulation Results
file_settings.campaign     = campaign;
file_settings.dyn_mode     = dyn_mode;
file_settings.input_bounds = input_bounds;
file_settings.backup       = backup;
file_description           = get_file_description(file_settings);
filename                   = strcat(file_description,'MonteCarlo_N',num2str(nTrials),'_Na',num2str(nAgents),'_Nnon',num2str(nNon),'.mat');
% save(filename)

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

input_txt = 'input_constraints';
if ~txt_settings.input_bounds
    input_txt = strcat('no_',input_txt);
end

backup_txt = 'backup';
if ~txt_settings.backup
    backup_txt = strcat('no_',backup_txt);
end

file_description = strcat('datastore/',campaign,'/',dyn_mode,'/',backup_txt,'/',input_txt,'/');

end
