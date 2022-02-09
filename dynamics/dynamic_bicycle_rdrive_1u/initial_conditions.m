% Load physical parameters
run('physical_params.m')
far = 10.0;
far = 15.0;
close = 5.0;
mc_start = 12.0;
mc_veloc = 6.0;

% This file contains simulation parameters related to environment
% tol = 5e-1;
tol = 1e-1;
       
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

% All equal
x1       = [ hlw -far   pi/2 5.0 0];
x2       = [-hlw  far  -pi/2 5.0 0];
x3       = [ far  hlw     pi 5.0 0];
x4       = [-far -hlw      0 5.0 0];

% Farther Back
x1       = [ hlw -far+3   pi/2 5.0 0];
x2       = [-hlw  far  -pi/2 5.0 0];
x3       = [ far+3  hlw     pi 5.0 0];
x4       = [-far -hlw      0 5.0 0];

% Up close
x1       = [ hlw -close-1   pi/2 5.0 0];
x2       = [-hlw  close  -pi/2 5.0 0];
x3       = [ close-1  hlw     pi 5.0 0];
x4       = [-close -hlw      0 5.0 0];

% Monte Carlo Start
x1       = [ hlw -mc_start   pi/2 mc_veloc 0];
x2       = [-hlw  mc_start  -pi/2 mc_veloc 0];
x3       = [ mc_start  hlw     pi mc_veloc 0];
x4       = [-mc_start -hlw      0 mc_veloc 0];


x0       = [x1; x2; x3; x4];
nAgents  = size(x0,1);
nStates  = size(x0,2);
              
% Goal Location Setup
xGoal{1} = [ hlw  -lw;
             hlw   0.0;
             0.0   hlw;
            -lw    hlw;
            -(far^2)   hlw];
xGoal{1} = [ hlw    -lw;
             hlw     lw;
             hlw     (far^2)]; 
xGoal{2} = [-hlw     lw;
            -hlw    -lw;
            -hlw    -(far^2)]; 
xGoal{3} = [ lw      hlw;
            -lw      hlw;
            -(far^2) hlw];
xGoal{4} = [-lw     -hlw;
             lw     -hlw;
             far^2  -hlw];
% xGoal{5} = [-lw     -hlw;
%              far^2  -hlw];
% xGoal{6} = [ lw      hlw;
%             -(far^2) hlw];
% xGoal{2} = [-hlw    -(far^2)]; 
% xGoal{3} = [-(far^2) hlw];
% xGoal{4} = [ far^2  -hlw];

% Segmented Paths -- time to complete each segment
first_T  = 2.0;
first_T  = 0.1;
last_T   = 8.0;
Tpath{1} = [first_T 1 2 1 last_T];
Tpath{1} = [1 first_T last_T];
Tpath{2} = [1 first_T last_T];
Tpath{3} = [1 first_T last_T];
Tpath{4} = [1 first_T last_T];

% Radius of each segment
Rpath{1} = [0 0 1.5 0 0];
Rpath{1} = [0 0 0];
Rpath{2} = [0 0 0];
Rpath{3} = [0 0 0];
Rpath{4} = [0 0 0];

% Type of each segment
path{1}  = {'linear', 'linear', 'circular_left', 'linear', 'linear'};
path{1}  = {'linear', 'linear', 'linear'};
path{2}  = {'linear', 'linear', 'linear'};
path{3}  = {'linear', 'linear', 'linear'};
path{4}  = {'linear', 'linear', 'linear'};

% xGoal{1} = [ hlw  -lw;
%              hlw   0.0;
%              0.0   hlw;
%             -(far^2)   hlw];
% xGoal{2} = [-hlw     lw;
%             -hlw    -(far^2)]; 
% xGoal{3} = [ lw      hlw;
%             -(far^2) hlw];
% xGoal{4} = [-lw     -hlw;
%              far^2  -hlw];
% 
% % Segmented Paths -- time to complete each segment
% Tpath{1} = [first_T 1 2 last_T];
% Tpath{2} = [1 last_T];
% Tpath{3} = [1 last_T];
% Tpath{4} = [1 last_T];
% 
% % Radius of each segment
% Rpath{1} = [0 0 1.5 0];
% Rpath{2} = [0 0];
% Rpath{3} = [0 0];
% Rpath{4} = [0 0];
% 
% % Type of each segment
% path{1}  = {'linear', 'linear', 'circular_left', 'linear'};
% path{2}  = {'linear', 'linear'};
% path{3}  = {'linear', 'linear'};
% path{4}  = {'linear', 'linear'};

gidx     = ones(size(x0,1),1);
xg       = zeros(size(x0,1),size(xGoal,3));
idx      = [1:1:nAgents];

Na       = nAgents;
Ns       = 0;
Nu       = nControls;
