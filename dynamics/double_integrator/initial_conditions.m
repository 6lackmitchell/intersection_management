% This file contains simulation parameters related to environment
tol = 5e-1;
       
% Three Vehicles:
% 1 -- Coming from south and turning West
% 2 Coming from North and proceeding straight South
% 3 Coming from East and proceeding straight Westt
x0           = [ 1.5 -13.0;
                -1.5  13.0;
                13.0   1.5];
xGoal    = {};
xGoal{1} = [  1.5  -3.0;
              1.5   0.0;
              0.0   1.5;
             -3.0   1.5;
            -13.0   1.5];
xGoal{2} = [ -1.5   3.0;
             -1.5 -13.0]; 
xGoal{3} = [  3.0   1.5;
            -13.0   1.5];

Tpath = [5 2 2 2 5;
         5 8;
         5 8];
Rpath = [0 0 1.5 0 0;
         0 0;
         0 0];
path = ['linear' 'linear' 'circular' 'linear' 'linear';
        'linear' 'linear';
        'linear' 'linear'];

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
