function [u] = analytical_fxt(t,x,settings)
%analytical_fxt - Controller based on analytical FxT Formulation
%This controller relies on Lyapunov analysis to design a control law in
%order to drive the state of the system to the desired trajectory in
%fixed-time and to remain there for all future time.
%
% Syntax:  [u] = analytical_fxt(t,x,settings)
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
% run('control_params.m')

Gamma = settings.Gamma;
e1    = settings.e1;
e2    = settings.e2;
T     = settings.T;
r     = settings.r;
rdot  = settings.rdot;

a     = 1;
b     = (1 - e1) / ((T * a * (1 - e1))*(e2 - 1));

% Here are the bones
xi  = r - x;
if norm(xi) > 0
    phi = (a * xi + b * xi * (xi * xi')) / norm(xi) * Gamma';
else
    phi = [0 0];
end
u   = rdot + phi;

end




%------------- END OF CODE --------------
