% Load physical parameters
run('physical_params.m')

% This file contains simulation parameters related to environment
tol = 1e-0;
       
% 6 Vehicles
x1       = [ hlw -far   pi/2 5 0];
x2       = [-hlw  far  -pi/2 5 0];
x3       = [ far  hlw     pi 5 0];
x4       = [-40.0 -hlw      0 5 0];
x4       = [-30.0 -hlw      0 5 0];
x5       = [-55.0 -hlw    0 5 0];
x6       = [ 45.0  hlw   pi 5 0];
x0       = [x1; x2; x3; x4];% x5; x6];
nAgents  = size(x0,1);
nStates  = size(x0,2);
              
% Goal Location Setup
xGoal{1} = [ hlw  -lw;
             hlw   0.0;
             0.0   hlw;
            -lw    hlw;
            -(far^2)   hlw];
xGoal{2} = [ -hlw  ien;
             -hlw -(far^2)]; 
xGoal{3} = [ ien     hlw;
            -(far^2) hlw];
xGoal{4} = [-ien    -hlw;
             far^2  -hlw];
xGoal{5} = [-ien    -hlw;
             far^2  -hlw];
xGoal{6} = [ ien     hlw;
            -(far^2) hlw];

% Segmented Paths -- time to complete each segment
Tpath{1} = [4 2 0.1 1 6];
Tpath{2} = [4 6];
Tpath{3} = [4 6];
Tpath{4} = [4.5 6];
Tpath{5} = [6 6];
Tpath{6} = [6 6];

% Radius of each segment
Rpath{1} = [0 0 1.5 0 0];
Rpath{2} = [0 0];
Rpath{3} = [0 0];
Rpath{4} = [0 0];
Rpath{5} = [0 0];
Rpath{6} = [0 0];

% Type of each segment
path{1}  = {'linear', 'linear', 'circular_left', 'linear', 'linear'};
path{2}  = {'linear', 'linear'};
path{3}  = {'linear', 'linear'};
path{4}  = {'linear', 'linear'};
path{5}  = {'linear', 'linear'};
path{6}  = {'linear', 'linear'};

gidx     = ones(size(x0,1),1);
xg       = zeros(size(x0,1),size(xGoal,3));
idx      = [1:1:nAgents];

Na       = nAgents;
Ns       = 0;
Nu       = nControls;
