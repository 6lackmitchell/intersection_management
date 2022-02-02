function u = lqr_tracking(t,x,aa,settings)
%lqr_tracking - Trajectory tracking controller using LQR.
% This controller tracks a time-varying reference trajectory by redefining
% the goal state at each point in time.
%
% Syntax:  [u] = lqr_tracking(t,x,aa,settings,control_params)
%
% Inputs:
%    t:        current time in sec -- float
%    x:        current state vector -- ROW vector
%    aa:       index of agent -- integer
%    settings: struct
%    control_params: struct
%
% Outputs:
%    u: nominal control input -- COLUMN vector
%
%
% Other m-files required: control_params.m, physical_params.m, timing.m
% Subfunctions: rotate
% MAT-files required: MAT files are generated for initial conditions
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% Jan 2022; Last revision: 31-Jan-2022
%------------- BEGIN CODE --------------% 


% Compute state error
xd     = settings.r(aa,1);
yd     = settings.r(aa,2);
xdotd  = settings.rdot(aa,1);
ydotd  = settings.rdot(aa,2);

xdes   = [xd; yd; xdotd; ydotd];
xerr   = x' - xdes;

% Double Integrator dynamics
A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0; 
     0 0 0 0];
B = [0 0;
     0 0;
     1 0;
     0 1];

% LQR Cost Function: x'*Q*x + u'*R*u
xy_gain = 5;
dot_gain = 10;
Q = eye(size(A,1));
Q(1:2,1:2) = xy_gain*Q(1:2,1:2);
Q(3:4,3:4) = dot_gain*Q(3:4,3:4);
R = eye(size(B,2));

% Compute LQR gain
[K,S,e] = lqr(A,B,Q,R);

% Compute optimal control
u = -K*xerr;
u = round(u,6);

end

function [newx_coord,newy_coord] = rotate(matrix,vector)
    z          = vector*matrix;
    newx_coord = z(1);
    newy_coord = z(2);
end

