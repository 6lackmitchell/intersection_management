% Load physical parameters
run('physical_params.m')

% This file contains simulation parameters related to environment
tol = 5e-1;
       
% 6 Vehicles
x1       = [  1.5 -13.0   pi/2 0];
x2       = [ -1.5  13.0 3*pi/2 0];
x3       = [ 13.0   1.5     pi 0];
x4       = [-13.0  -1.5      0 0];
x5       = [-20.0  -1.5      0 0];
x6       = [ 10.0   1.5     pi 0];
x0       = [x1; x2; x3];%; x4; x5; x6];
nAgents  = size(x0,1);
nStates  = size(x0,2);
              
% Goal Location Setup
xGoal{1} = [  1.5  -3.1  pi/2;
              1.5   0.0  pi/2;
              0.0   1.5  pi;
             -3.0   1.5  pi;
            -13.0   1.5  pi];
xGoal{2} = [ -1.5   3.1  3*pi/2;
             -1.5 -13.0  3*pi/2]; 
xGoal{3} = [  3.1   1.5  pi;
            -13.0   1.5  pi];
xGoal{4} = [ 13.0  -1.5  0];
xGoal{5} = [ 13.0  -1.5  0];
xGoal{6} = [-13.0   1.5  pi];

% Segmented Paths -- time to complete each segment
Tpath{1} = [5 2 2 2 5];
Tpath{2} = [5 5];
Tpath{3} = [5 5];
Tpath{4} = [10];
Tpath{5} = [10];
Tpath{6} = [15];

% Radius of each segment
Rpath{1} = [0 0 1.5 0 0];
Rpath{2} = [0 0];
Rpath{3} = [0 0];
Rpath{4} = [0];
Rpath{5} = [0];
Rpath{6} = [0];

% Type of each segment
path{1}  = {'linear', 'linear', 'circular_left', 'linear', 'linear'};
path{2}  = {'linear', 'linear'};
path{3}  = {'linear', 'linear'};
path{4}  = {'linear'};
path{5}  = {'linear'};
path{6}  = {'linear'};

gidx     = ones(size(x0,1),1);
xg       = zeros(size(x0,1),size(xGoal,3));
idx      = [1:1:nAgents];

Na       = nAgents;
Ns       = 0;
Nu       = nControls;
