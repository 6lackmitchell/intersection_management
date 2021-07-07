% This file contains simulation parameters related to control
%run('clfs/clfs.m')
run('../../../settings/environment.m')

% Get CLF/CBF handles
[V,dVdx] = clfs();
[B,dBdx] = cbfs();

% Control Constraint
u_max = 5.0;

% Safe Distance
R = 1.0;

% N-values
Nu = 2;
Np = 1;
Ns = 4;
nCBFs = 4 + nAgents;

% QP Control Parameters
q = [1; 1; 100; 1; 1; 1; 1];

% FxTS Parameters
c1 = 1;
c2 = 1;
e1 = 0.5;
e2 = 1.5;

% Ellipses
cx1 = 1;   cx2 =  2;  cx3 =  -1; cx4 = -2;
cy1 = 2;   cy2 = -1;  cy3 =  -2; cy4 =  1;
dx1 = 1.5; dx2 =  1;  dx3 = 1.5; dx4 =  1;
dy1 = 1;   dy2 =  1.5; dy3 = 1; dy4 = 1.5;

% CLF Weight
W = eye(length(2));

% Alpha Function
alph = 1;
