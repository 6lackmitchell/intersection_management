% Load physical parameters
run('physical_params.m')

% This file contains simulation parameters related to environment
tol = 5e-1;
       
% 6 Vehicles
v0       = 0.0;
x1       = [    hlw    -far  pi/2 v0 0];
x2       = [   -hlw     far -pi/2 v0 0];
x3       = [    far     hlw    pi v0 0];
x4       = [   -far    -hlw     0 v0 0];
x5       = [-far-10    -hlw     0 v0 0];
x6       = [ far+10     hlw    pi v0 0];
x7       = [   -hlw  far+10    pi v0 0];
x8       = [    hlw -far-10  pi/2 v0 0];
x9       = [-far-20    -hlw     0 v0 0];
x10      = [ far+20     hlw    pi v0 0];
x0       = [x1; x2; x3; x4; x5; x6; x7; x8; x9; x10];
nAgents  = size(x0,1);
nStates  = size(x0,2);

% Segmented Paths -- time to complete each segment
Tpath{1}  = [4 1 2 1 6];
Tpath{2}  = [4 1 2 1 6];
Tpath{3}  = [4 6];
Tpath{4}  = [4 1 2 1 6];
Tpath{5}  = [6 6];
Tpath{6}  = [6 1 2 1 6];
Tpath{7}  = [6 6];
Tpath{8}  = [6 1 2 1 6];
Tpath{9}  = [8 6];
Tpath{10} = [8 6];
              
% Goal Location Setup
xGoal{1}  = [   hlw    -lw;
                hlw    0.0;
                0.0    hlw;
                -lw    hlw;
             -far^2    hlw];
xGoal{2}  = [  -hlw     lw;
               -hlw    0.0;
                0.0   -hlw;
                 lw   -hlw;
              far^2   -hlw;]; 
xGoal{3}  = [    lw    hlw;
             -far^2    hlw];
xGoal{4}  = [   -lw   -hlw;
                0.0   -hlw;
                hlw    0.0;
                hlw     lw;
                hlw  far^2];
xGoal{5}  = [   -lw   -hlw;
              far^2   -hlw];
xGoal{6}  = [    lw    hlw;
                0.0    hlw;
               -hlw    0.0;
               -hlw    -lw;
               -hlw -far^2];
xGoal{7}  = [  -hlw    -lw;
               -hlw  far^2];
xGoal{8}  = [   hlw    -lw;
                hlw    0.0;
                0.0    hlw;
                -lw    hlw;
             -far^2    hlw];
xGoal{9}  = [   -lw   -hlw;
              far^2   -hlw];
xGoal{10} = [    lw    hlw;
             -far^2    hlw];

% Radius of each segment
Rpath{1}  = [0 0 1.5 0 0];
Rpath{2}  = [0 0 1.5 0 0];
Rpath{3}  = [0 0];
Rpath{4}  = [0 0 1.5 0 0];
Rpath{5}  = [0 0];
Rpath{6}  = [0 0 1.5 0 0];
Rpath{7}  = [0 0];
Rpath{8}  = [0 0 1.5 0 0];
Rpath{9}  = [0 0];
Rpath{10} = [0 0];

% Type of each segment
path{1}  = {'linear', 'linear', 'circular_left', 'linear', 'linear'};
path{2}  = {'linear', 'linear', 'circular_left', 'linear', 'linear'};
path{3}  = {'linear', 'linear'};
path{4}  = {'linear', 'linear', 'circular_left', 'linear', 'linear'};
path{5}  = {'linear', 'linear'};
path{6}  = {'linear', 'linear', 'circular_left', 'linear', 'linear'};
path{7}  = {'linear', 'linear'};
path{8}  = {'linear', 'linear', 'circular_left', 'linear', 'linear'};
path{9}  = {'linear', 'linear'};
path{10} = {'linear', 'linear'};

gidx     = ones(size(x0,1),1);
xg       = zeros(size(x0,1),size(xGoal,3));
idx      = [1:1:nAgents];

Na       = nAgents;
Ns       = 0;
Nu       = nControls;
