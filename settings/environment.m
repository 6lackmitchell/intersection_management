% This file contains simulation parameters related to environment
tol = 5e-1;

% Initial conditions
x0     = [ 5  0;
           0  5;
          -5  0;
           0 -5];
x_goal = [-5  0;
           0 -5;
           5  0;
           0  5];
       
% Two Vehicles -- One crossing straight through, one turning left
x0           = [ 1.5 -13.0;
                -1.5  13.0];
xGoal(1,:,:) = [ 1.5 -3.0; -3.0  1.5; -13.0   1.5];
xGoal(2,:,:) = [-1.5  3.0; -1.5 -3.0; - 1.5 -13.0];
gidx         = ones(size(x0,1),1);
xg           = zeros(size(x0));

% % Double Integrator
% x0     = [ 5  0 0 0;
%            0  5 0 0;
%           -5  0 0 0;
%            0 -5 0 0];
% x_goal = [-5  0 0 0;
%            0 -5 0 0;
%            5  0 0 0;
%            0  5 0 0];

nLanes = 1; 
nWay   = 4;

nAgents   = size(x0,1);
nStates   = size(x0,2);
nControls = 2;
nCBFs     = 4 + (nAgents-1);
idx       = [1:1:nAgents];
uLast     = zeros(nAgents,nControls);

