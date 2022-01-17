function u = ailon2020_kb_tracking_fxts(t,x,aa,settings,control_params)
%ailon2020_kb_tracking_fxts - FxTS tracking controller for bicycle model
%This controller drives the state of the system governed by kinematic
%bicycle dynamics to some desired trajectory in fixed-time (and keeps it on
%the trajectory).
%
% Syntax:  [u] = ailon2020_kb_tracking_fxts(t,x,r,rdot,r2dot,t0,aa)
%
% Inputs:
%    t:        current time in sec -- float
%    x:        current state vector -- ROW vector
%    r:        desired state vector -- ROW vector
%    rdot:     desired velocity vector -- ROW vector
%    r2dot:    desired acceleration vector -- ROW vector
%    t0:       initial time -- float
%    aa:       index of agent -- integer
%
% Outputs:
%    u: nominal control input -- COLUMN vector
%
% Example: 
%    u = ailon2020_kb_tracking_fxts(t,x,r,rdot,r2dot,t0,aa)
%
% Other m-files required: control_params.m, physical_params.m, timing.m
% Subfunctions: rotate
% MAT-files required: MAT files are generated for initial conditions
%
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% Aug 2021; Last revision: 13-Sep-2021
%------------- BEGIN CODE --------------% 
% Load control params

% control_params = load('control_params.mat')
% run('control_params.m')
% run('physical_params.m')
% run('timing.m')
precision = 2;

% Parse out desired trajectory
beta   = x(5);
xd     = settings.r(aa,1);
yd     = settings.r(aa,2);
xdotd  = settings.rdot(aa,1);
ydotd  = settings.rdot(aa,2);
x2dotd = settings.r2dot(aa,1);
y2dotd = settings.r2dot(aa,2);
phid   = atan2(ydotd,xdotd);
vd     = sqrt(xdotd^2 + ydotd^2);
v      = x(4) / cos(beta);

err1 = round(x(1) - xd,12);
err2 = round(x(2) - yd,12);

% % Determine initial control conditions
% filepath = '/Users/mblack/Documents/git/intersection_management/controllers/pcca/';
% filename = strcat(filepath,'initial_tracking_conditions',num2str(aa),'.mat');
% if t == t0
%     v0  = v;
%     px0 = xdotd - 2*v - control_params.k1*err1^2;
%     py0 = ydotd - 2*v - control_params.k2*err2^2;
%     save(filename,'v0','px0','py0')
% else
%     load(filename,'v0','px0','py0')
% end

% Need to apply a rotation here if the trajectory cannot be represented by
% a function
rotation_needed = (x(3) > pi/4 && x(3) < 3*pi/4) || (x(3) < -pi/4 && x(3) > -3*pi/4);
if rotation_needed
    % 90 deg rotation to right
    rot    = [0 -1; 1 0];
    x(1:2) = x(1:2)*rot;
    x(3)   = wrapToPi(x(3) - pi/2);
    phid   = wrapToPi(phid - pi/2);
    
%     [px0,py0]       = rotate(rot,[px0 py0]);
    [err1,err2]     = rotate(rot,[err1 err2]);
    [xdotd,ydotd]   = rotate(rot,[xdotd ydotd]);
    [x2dotd,y2dotd] = rotate(rot,[x2dotd y2dotd]);
    
    newk1 = control_params.k2;
    control_params.k2 = control_params.k1;
    control_params.k1 = newk1;
    
end

betad = -phid + atan2(round(ydotd,precision),round(xdotd,precision));
betad = wrapToPi(betad);

px   = xdotd - control_params.k1*err1;
py   = ydotd - control_params.k2*err2;
norm_p = sqrt(px^2 + py^2);

x_dot   = v *(cos(x(3)) - sin(x(3))*tan(x(5)));
xd_dot  = vd*(cos(phid) - sin(phid)*tan(betad));
y_dot   = v *(sin(x(3)) + cos(x(3))*tan(x(5)));
yd_dot  = vd*(sin(phid) + cos(phid)*tan(betad));


err1dot = x_dot - xd_dot;
err2dot = y_dot - yd_dot;
pxdot   = x2dotd - control_params.k1*err1dot;
pydot   = y2dotd - control_params.k2*err2dot;

theta_s    = atan2(round(py,precision),round(px,precision));
theta_sdot = (pydot*px - pxdot*py) / norm_p^2;

err3    = wrapToPi((x(3) + x(5)) - theta_s);
err3dot = -sign(err3)*(control_params.a1*abs(err3)^(0.5) + control_params.a2*abs(err3)^(1.5));

betadot = err3dot + theta_sdot - v*tan(x(5))/settings.Lr;
betadot = round(betadot,2);
% beta    = beta + betadot*dt;
% tand    = round((Lr+Lf)/Lr * tan(x(5)),2);

% norm_p0 = sqrt(px0^2 + py0^2);
% c0      = v0 - norm_p0;

if err1 == 0 && err2 == 0
    Bdot = 0;
else
    M = -control_params.a3*sqrt(err1^2 + err2^2) - control_params.a4*(err1^4 + 2*err1^2*err2^2 + err2^4);
    N = err1*px + err2*py;
    B = M / N * norm_p; % Unused, but written to obtain Bdot

    Mdot = -control_params.a3*(err1*err1dot + err2*err2dot)/sqrt(err1^2 + err2^2) - control_params.a4*(4*err1^3*err1dot + 4*err1*err1dot*err2^2 + 4*err1^2*err2*err2dot + 4*err2^3*err2dot);
    Ndot = err1dot*px + err1*pxdot + err2dot*py + err2*pydot;
    Bdot = (M/N)*((px*pxdot+py*pydot)/norm_p) + ((Mdot*N - M*Ndot)/N^2)*norm_p;
end
a       = (px*pxdot + py*pydot) / norm_p + Bdot;

% Thus far we have computed acceleration for the velocity of the c.g. of
% the bicycle -- we need to translate it into acceleration for the velocity
% of the rear wheel
ar = a*cos(beta) - x(4)*betadot*tan(beta);

u = [betadot; ar];
end

function [newx_coord,newy_coord] = rotate(matrix,vector)
    z          = vector*matrix;
    newx_coord = z(1);
    newy_coord = z(2);
end

