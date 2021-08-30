function u = ailon2020_kb_tracking(t,x,r,rdot,r2dot,t0,aa)
%AILON2020_KB_TRACKING Asymptotically stable tracking controller for 
%kinematic bicycle model
%   Detailed explanation goes here
% Load control params
run('control_params.m')
run('physical_params.m')
run('timing.m')
precision = 2;

% Parse out desired trajectory
beta   = x(5);
xd     = r(1);     yd     = r(2);
xdotd  = rdot(1);  ydotd  = rdot(2);
x2dotd = r2dot(1); y2dotd = r2dot(2);
phid   = atan2(ydotd,xdotd);
vd     = sqrt(xdotd^2 + ydotd^2);
v      = x(4) / cos(beta);

err1 = x(1) - xd;
err2 = x(2) - yd;

% Determine initial control conditions
filepath = '/Users/mblack/Documents/git/intersection_management/controllers/rdrive/';
filename = strcat(filepath,'initial_tracking_conditions',num2str(aa),'.mat');
if t == t0
    v0  = v;
    px0 = xdotd - a1*tanh(k1*err1);
    py0 = ydotd - a2*tanh(k2*err2);
    save(filename,'v0','px0','py0')
else
    load(filename)
end

% Need to apply a rotation here if the trajectory cannot be represented by
% a function
if (x(3) > pi/4 && x(3) < 3*pi/4) || (x(3) < -pi/4 && x(3) > -3*pi/4)
    % 90 deg rotation to right
    rot    = [0 -1; 1 0];
    x(1:2) = x(1:2)*rot;
    x(3)   = wrapToPi(x(3) - pi/2);
    phid   = wrapToPi(phid - pi/2);
    
    [px0,py0]       = rotate(rot,[px0 py0]);
    [err1,err2]     = rotate(rot,[err1 err2]);
    [xdotd,ydotd]   = rotate(rot,[xdotd ydotd]);
    [x2dotd,y2dotd] = rotate(rot,[x2dotd y2dotd]);
    
    newa1 = a2;
    a2 = a1;
    a1 = newa1;
    
end

betad = -phid + atan2(round(ydotd,precision),round(xdotd,precision));
betad = wrapToPi(betad);

px   = xdotd - a1*tanh(k1*err1);
py   = ydotd - a2*tanh(k2*err2);
norm_p = sqrt(px^2 + py^2);

err1dot = v*cos(x(3)+beta) - vd*cos(phid+betad);
err2dot = v*sin(x(3)+beta) - vd*sin(phid+betad);
pxdot   = x2dotd + a1*k1*(tanh(k1*err1)^2-1)*err1dot;
pydot   = y2dotd + a2*k2*(tanh(k2*err2)^2-1)*err2dot;

theta_s    = atan2(round(py,precision),round(px,precision));
theta_sdot = (pydot*px - pxdot*py) / norm_p;

norm_p0 = sqrt(px0^2 + py0^2);
c0 = v0 - norm_p0;
a  = (px*pxdot + py*pydot) / norm_p - c0*pp*exp(-pp*(t-t0));
% a  = (px*pxdot + py*pydot) / norm_p - c0*pp;%*exp(-pp/10*(t-t0));

err3    = wrapToPi((x(3) + beta) - theta_s);
betadot = theta_sdot - v*sin(beta)/Lr - a3*tanh(k3*err3);
beta    = beta + betadot*dt;
tand    = round((Lr+Lf)/Lr * tan(beta),2);

% Thus far we have computed acceleration for the velocity of the c.g. of
% the bicycle -- we need to translate it into acceleration for the velocity
% of the rear wheel
ar = a*cos(beta) - x(4)*betadot*tan(beta);

u = [tand; ar];
end

function [newx_coord,newy_coord] = rotate(matrix,vector)
    z          = vector*matrix;
    newx_coord = z(1);
    newy_coord = z(2);
end

