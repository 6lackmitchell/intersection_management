function [r,rdot,rddot] = trajectories(t,x,settings)
%trajectories - Desired trajectories for cars in/approaching intersection
%This
%
% Syntax:  [u] = trajectories(t,x,settings)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%    settings - structure object containing additional settings
%
% Outputs:
%    u - control input - ROW vector
%
% Example: 
%    [u(k+1,:),d] = analytical_fxt(t,x,settings)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% Aug 2021; Last revision: 2-Aug-2021
%------------- BEGIN CODE --------------
epsilon = 0.5;

T    = settings.T;
t0   = settings.t0;
xS   = settings.xS;
xF   = settings.xF;
th0  = settings.th0;
R    = settings.R;
path = settings.path;
tau  = min((t - t0) / T,1);

%**%

if strcmp(path,'linear')
    r    = xF * tau + xS * (1 - tau);
    
    if tau < 1
        rdot = (xF - xS) / T;
    elseif norm(r - x(1:2)) > epsilon
        rdot = (r - x(1:2)) / 0.1;
    else
        rdot = [0 0];
    end
    
    rddot = [0 0];
    
elseif contains(path,'circular')
    
    if contains(path,'left')
        x0   = xS(1:2) + [-R*sin(th0) R*cos(th0)];
        phi0 = th0 - (pi / 2);
        phif = th0;
    elseif contains(path,'right')
        x0 = xS(1:2) + [R*sin(th0) -R*cos(th0)];
        phi0 = th0 + (pi / 2);
        phif = th0;
    end
    
    thdot  = (phif - phi0) / T;
    thddot = 0;
    th     = phif * tau + phi0 * (1 - tau);
    r      = x0+[R*cos(th) R*sin(th)];
    rdot   = [-R*thdot*sin(th) R*thdot*cos(th)];
    rddot  = [-R*(thddot*sin(th) + thdot^2*cos(th));
               R*(thddot*cos(th) - thdot^2*sin(th))]';
    
end

%**%

end




%------------- END OF CODE --------------
