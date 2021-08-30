function [f_handle,g_handle] = kinematic_bicycle_rdrive()
%double_integrator - State-derivative of double-integrator dynamics
%For this iteration of the double-integrator dynamics, there will be N
%vehicles each of which only have acceleration control over one of their
%principal directions (i.e. x or y). This only represents the dynamics for 
%one of the vehicles.
%
% Syntax:  [xdot] = double_integrator(t,x,u)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%    u - control input -- ROW vector
%
% Outputs:
%    xdot - vector of state-derivatives - ROW vector
%
% Example: 
%    x(k+1) = x(k) + dt*double_integrator(t,x,u)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% May 2021; Last revision: 29-May-2021
%------------- BEGIN CODE --------------
f_handle = @f;
g_handle = @g;

end

function vec = f(t,x)
%double_integrator - State-derivative of double-integrator dynamics
%For this iteration of the double-integrator dynamics, there will be N
%vehicles each of which only have acceleration control over one of their
%principal directions (i.e. x or y). This only represents the dynamics for 
%one of the vehicles.
%
% Syntax:  [xdot] = double_integrator(t,x,u)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%    u - control input -- ROW vector
%
% Outputs:
%    xdot - vector of state-derivatives - ROW vector
%
% Example: 
%    x(k+1) = x(k) + dt*kinematic_bicycle(t,x,u)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% Aug 2021; Last revision: 19-Aug-2021
%------------- BEGIN CODE --------------
vec = [0;
       0;
       0; 
       0]; % No drift term

end

function vec = g(t,x)
%double_integrator - State-derivative of double-integrator dynamics
%For this iteration of the double-integrator dynamics, there will be N
%vehicles each of which only have acceleration control over one of their
%principal directions (i.e. x or y). This only represents the dynamics for 
%one of the vehicles.
%
% Syntax:  [xdot] = double_integrator(t,x,u)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%    u - control input -- ROW vector
%
% Outputs:
%    xdot - vector of state-derivatives - ROW vector
%
% Example: 
%    x(k+1) = x(k) + dt*kinematic_bicycle(t,x,u)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% Aug 2021; Last revision: 20-Aug-2021
%------------- BEGIN CODE --------------
l   = 2; % length between front and rear wheels
vec = [cos(x(3))        0;
       sin(x(3))        0;
       0                1;
       sin(x(3)-x(4))/l 0]; % Controls are vel and steering angular vel

end



%------------- END OF CODE --------------
