% Load physical parameters
run('physical_params.m')

% This file contains simulation parameters related to environment
tol = 5e-1;
       
% 6 Vehicles
x1       = [ hlw -far   pi/2 5];
x2       = [-hlw  far  -pi/2 5];
x3       = [ far  hlw     pi 5];
x4       = [-80.0 -hlw      0 5];
x5       = [-100.0  -hlw    0 5];
x6       = [ 20.0   hlw   pi 5];
x0       = [x1; x2; x3; x4; x5; x6];
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
xGoal{3} = [ ien   hlw;
            -(far^2)   hlw];
xGoal{4} = [ far^2  -hlw];
xGoal{5} = [ far^2  -hlw];
xGoal{6} = [-(far^2)   hlw];

% Segmented Paths -- time to complete each segment
Tpath{1} = [4 2 2 2 10];
Tpath{2} = [4 10];
Tpath{3} = [4 10];
Tpath{4} = [30];
Tpath{5} = [30];
Tpath{6} = [30];

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
