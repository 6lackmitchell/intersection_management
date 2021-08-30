% This file contains simulation parameters related to control
% cwd = pwd;
% run(strcat(cwd,'../settings/environment.m'))
% run('../settings/environment.m')
run('kinematic_bicycle_env.m')
% run('cbfs.m')
% run('clfs.m')

% Get CLF/CBF handles
[V,dVdx] = clfs();
[B,dBdx] = cbfs();

% Control Constraint
tau_max = pi / 4;
acc_max = 2 * 9.81;

% Tracking Control Params
Tc = 0.25;

% Safe Distance
R = 2.0;

% N-values
Nu = 2;
Np = 0;
Ns = 0;%4 + (nAgents-1);

% QP Control Parameters
q = [1 / tau_max^2; 1 / acc_max^2];
q = repmat(q,3,1);
% q = [q; ones(Ns,1)];

% FxTS Parameters
Gamma = [1 0; 0 1];
e1 = 0.5;
e2 = 1.5;
c1 = 1;

% Ellipses
cx1 = 1;   cx2 =  2;  cx3 =  -1; cx4 = -2;
cy1 = 2;   cy2 = -1;  cy3 =  -2; cy4 =  1;
dx1 = 1.5; dx2 =  1;  dx3 = 1.5; dx4 =  1;
dy1 = 1;   dy2 =  1.5; dy3 = 1; dy4 = 1.5;

% CLF Weight
W = eye(length(2));

% Alpha Function
alph = 1;
