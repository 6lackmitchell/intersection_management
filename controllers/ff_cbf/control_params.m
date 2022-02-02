% This file contains simulation parameters related to control
clear; clc;

% Load relevant parameters
run('initial_conditions_cpca.m');

% Tracking Control Params
Tc = 0.25;

% Safe Distance
R = 2.0;

% QP Control Parameters
% q = ones(Nu+Ns,1);
q = [100 / umax(1)^2; 1 / umax(2)^2]; % This works well at tmax=3
qu = [1e0 / umax(1)^2; 1e-4 / umax(2)^2]; % This works well in general
qg = [1];

% FxTS Parameters
Gamma = [1 0; 0 1];
c1 = 1;
c2 = 1;
e1 = 0.5;
e2 = 1.5;

% Tracking Control Parameters
a1 = 0.4; a2 = a1; a3 = 2*a1; a4 = 2*a1; 
k1 = 2.0; k2 = 2.0;

% Mode
mode = 'kinematic';

% Save to .mat file for future loading
filename = 'control_params.mat';
save(filename)
