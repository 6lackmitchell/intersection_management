% Load physical parameters
run('physical_params.m')
far = 10.0;

% This file contains simulation parameters related to environment
tol = 5e-1;
       
% 6 Vehicles
x1       = [ hlw -far   pi/2 5.0 0];
x2       = [-hlw  far  -pi/2 5.0 0];
x3       = [ far  hlw     pi 5.0 0];
x4       = [-far -hlw      0 5.0 0];
x4       = [-far+1.5 -hlw      0 5.0 0];

x1       = [ hlw     -far+0.5   pi/2 6.0 0];
x2       = [-hlw      far+0.5  -pi/2 5.0 0];
x3       = [ far-1    hlw         pi 4.0 0];
x4       = [-far+1.5 -hlw          0 5.0 0];
x4       = [-far+3.0 -hlw          0 5.0 0];

% Blue Behind
x1       = [ hlw -far   pi/2 5.0 0];
x2       = [-hlw  far  -pi/2 5.0 0];
x3       = [ far  hlw     pi 5.0 0];
x4       = [-far -hlw      0 7.5 0];

% Blue Ahead
x1       = [ hlw -far   pi/2 5.0 0];
x2       = [-hlw  far  -pi/2 5.0 0];
x3       = [ far  hlw     pi 5.0 0];
x4       = [-far -hlw      0 5.0 0];

% Identical initial conditions
x1       = [ hlw -far   pi/2 5.0 0];
x2       = [-hlw  far  -pi/2 5.0 0];
x3       = [ far  hlw     pi 5.0 0];
x4       = [-far -hlw      0 5.0 0];

% Blue Ahead
x1       = [ hlw -far+2   pi/2 5.0 0];
x2       = [-hlw  far  -pi/2 5.0 0];
x3       = [ far  hlw     pi 5.0 0];
x4       = [-far -hlw      0 5.0 0];

% % Pure Test
% x1       = [ hlw -3.5   pi/2 1.0 0];
% x2       = [-hlw  far  -pi/2 5.0 0];
% x3       = [ far  hlw     pi 5.0 0];
% x4       = [-1.5 -hlw      0 1.0 0];

% x1       = [ hlw -far+1   pi/2 5.1 0];
% x2       = [-hlw  far-2  -pi/2 5.2 0];
% x3       = [ far-1  hlw     pi 4.9 0];
% x4       = [-far-1 -hlw      0 4.8 0];

% x2       = [-hlw  4.5  -pi/2 5.0 0];
% x4       = [-13.0+(24223)/(262144) -hlw      0 5.0 0]; % Go
% x4       = [-13.0+(24224)/(262144) -hlw      0 5.0 0]; % Wait

% % x4       = [-50.0 -hlw      0 5 0];
% x5       = [-55.0 -hlw    0 5 0];
% x6       = [ 45.0  hlw   pi 5 0];
x0       = [x1; x2; x3; x4];%; x5; x6];%; x7; x8; x9; x10];
% x0       = [x1; x4];%; x5; x6];%; x7; x8; x9; x10];
nAgents  = size(x0,1);
nStates  = size(x0,2);
              
% Goal Location Setup
xGoal{1} = [ hlw  -lw;
             hlw   0.0;
             0.0   hlw;
            -lw    hlw;
            -(far^2)   hlw];
xGoal{2} = [-hlw     lw;
            -hlw    -(far^2)]; 
xGoal{3} = [ lw      hlw;
            -(far^2) hlw];
xGoal{4} = [-lw     -hlw;
             far^2  -hlw];
% xGoal{5} = [-lw     -hlw;
%              far^2  -hlw];
% xGoal{6} = [ lw      hlw;
%             -(far^2) hlw];
xGoal{2} = [-hlw    -(far^2)]; 
xGoal{3} = [-(far^2) hlw];
xGoal{4} = [ far^2  -hlw];

% Segmented Paths -- time to complete each segment
first_T  = 1.0;
last_T   = 10.0;
last_T   = 8.0;
Tpath{1} = [first_T 1 3 1 last_T];
Tpath{1} = [first_T 1 2 1 last_T];
Tpath{1} = [first_T 0.5 1.5 0.5 last_T];
Tpath{2} = [first_T last_T];
Tpath{3} = [first_T last_T];
Tpath{2} = [last_T];
Tpath{3} = [last_T];

% % Experimental
Tpath{4} = [first_T last_T];
Tpath{4} = [first_T 1];
Tpath{4} = [last_T];


Tpath{5} = [6 6];
Tpath{6} = [6 6];

% Radius of each segment
Rpath{1} = [0 0 1.5 0 0];
Rpath{2} = [0 0];
Rpath{3} = [0 0];
Rpath{4} = [0 0];
Rpath{2} = [0];
Rpath{3} = [0];
Rpath{4} = [0];
Rpath{5} = [0 0];
Rpath{6} = [0 0];

% Type of each segment
path{1}  = {'linear', 'linear', 'circular_left', 'linear', 'linear'};
path{2}  = {'linear', 'linear'};
path{3}  = {'linear', 'linear'};
path{4}  = {'linear', 'linear'};
path{2}  = {'linear'};
path{3}  = {'linear'};
path{4}  = {'linear'};
path{5}  = {'linear', 'linear'};
path{6}  = {'linear', 'linear'};

% xGoal{2} = [ far^2  -hlw];
% Tpath{2} = [last_T];
% Rpath{2} = [0];
% path{2}  = {'linear'};

gidx     = ones(size(x0,1),1);
xg       = zeros(size(x0,1),size(xGoal,3));
idx      = [1:1:nAgents];

Na       = nAgents;
Ns       = 0;
Nu       = nControls;
