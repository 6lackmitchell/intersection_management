function [u,reached] = lqr_tracking(t,x,aa)
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

% LQR X
xlqr = [x(1); x(2); x(4)*cos(x(3)); x(4)*sin(x(3))];


% Compute desired position
[xd,yd,Q,reached] = get_desired_position(t,x,aa);

% Compute desired velocity
perr   = x(1:2)' - [xd;yd];
Av = zeros(2);
Bv = eye(2);
Qv = eye(2);
Rv = eye(2);
[Kv,~,~] = lqr(Av,Bv,Qv,Rv);
vd = -Kv*perr;

xdes   = [xd; yd; vd];
xerr   = xlqr - xdes;

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
xy_gain    = 2;
Q(1:2,1:2) = xy_gain*Q(1:2,1:2);
R = eye(size(B,2));

% Compute LQR gain
[K,~,~] = lqr(A,B,Q,R);

% Compute optimal control
u = axay_to_wa(-K*xerr,x);
u = round(u,6);

end

function [unew] = axay_to_wa(u,x)

if abs(x(4)) > 0.1
    S = [-x(4)*sin(x(3))*sec(x(5))^2 cos(x(3))-sin(x(3))*tan(x(5)); ...
          x(4)*cos(x(3))*sec(x(5))^2 sin(x(3))+cos(x(3))*tan(x(5))];

    phidot = x(4)*tan(x(5));
    xdot   = x(4)*(cos(x(3))-sin(x(3))*tan(x(5)));
    ydot   = x(4)*(sin(x(3))+cos(x(3))*tan(x(5)));

    d = [u(1) + ydot*phidot; u(2) - xdot*phidot];

    unew = S\d;

else
    unew = [0; sqrt(u(1)^2+u(2)^2)];

end

end
