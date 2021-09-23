% This file contains simulation parameters related to control
% cwd = pwd;
% run(strcat(cwd,'../settings/environment.m'))
% run('../settings/environment.m')
% run('kinematic_bicycle_env.m')
% run('cbfs.m')
% run('clfs.m')

% % Get CLF/CBF handles
% [V,dVdx] = clfs();
% [B,dBdx] = cbfs();

% % Control Limits
% tau_max = pi / 4;
% acc_max = 2 * 9.81;

% Load relevant parameters
run('vehicle10_initial_conditions.m')

% Tracking Control Params
Tc = 0.25;

% Safe Distance
R = 2.0;

% QP Control Parameters
% q = ones(Nu+Ns,1);
q = [100 / umax(1)^2; 1 / umax(2)^2]; % This works well at tmax=3
q = [1e0 / umax(1)^2; 1e-4 / umax(2)^2];
% q = [20; 0.1];

% FxTS Parameters
Gamma = [1 0; 0 1];
c1 = 1;
c2 = 1;
e1 = 0.5;
e2 = 1.5;

% Tracking Control Parameters
a1 = 0.5; a2 = 2.0; a3 = 10.0; % This was working 1:28PM Sep 1
k1 = 1.0; k2 = 1.0; k3 = 1.0;  % This was working 1:28PM Sep 1

a1 = 1.0; a2 = 1.0; a3 = 50.0; a4 = 20.0; % This WORKED FULLY 3:15PM Sep 1
k1 = 2.0; k2 = 2.0;

% a1 = 2.0; a2 = 2.0; a3 = 20.0; a4 = 200.0; % Experimental
% k1 = 5.0; k2 = 5.0;

% a1 = 2.0; a2 = 2.0; a3 = 100.0; a4 = 100.0; % Also worked 3:25PM Sep 1
% k1 = 2.0; k2 = 2.0;
% 


% a1 = 2.0; a2 = 2.0; a3 = 100.0; a4 = 100.0; % Experimental
% k1 = 0.1; k2 = 0.1;

% a1 = 0.2; a2 = 0.2; a3 = 100.0; a4 = 100.0; % Experimental
% 
% Start low and go higher
a1 = 10.0; a2 = 10.0; a3 = 0.1; a4 = 0.1; % Experimental
k1 = 20.0; k2 = 20.0;

% These work -- 8:24AM Sep 10
a1 = 1.0; a2 = 1.0; a3 = 1.0; a4 = 1.0; % Experimental
k1 = 10.0; k2 = 10.0;

% Start low and go higher
a1 = 2.0; a2 = 2.0; a3 = 1.0; a4 = 1.0; % Experimental
k1 = 2.0; k2 = 2.0;

% THESE WORK REALLY WELL -- Sep 13 5:23PM
a1 = 0.1; a2 = 0.1; a3 = 0.1; a4 = 0.1; % Experimental
k1 = 2.0; k2 = 2.0;

% Mode
mode = 'kinematic';

% % Ellipses
% cx1 = 1;   cx2 =  2;  cx3 =  -1; cx4 = -2;
% cy1 = 2;   cy2 = -1;  cy3 =  -2; cy4 =  1;
% dx1 = 1.5; dx2 =  1;  dx3 = 1.5; dx4 =  1;
% dy1 = 1;   dy2 =  1.5; dy3 = 1; dy4 = 1.5;

% % CLF Weight
% W = eye(length(2));

% % Alpha Function
% alph = 1;
