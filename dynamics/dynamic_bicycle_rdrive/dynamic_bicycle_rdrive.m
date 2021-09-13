function [f_handle,g_handle] = dynamic_bicycle_rdrive()
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
% Sep 2021; Last revision: 8-Sep-2021
%------------- BEGIN CODE --------------
run('physical_params.m')

vec = [x(4)*(cos(x(3)) - sin(x(3))*tan(x(5)));
       x(4)*(sin(x(3)) + cos(x(3))*tan(x(5)));
       x(4)*tan(x(5))/Lr; 
       0;
       0];

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
% Sep 2021; Last revision: 8-Sep-2021
%------------- BEGIN CODE --------------

% bterm = cos(x(5))^2 * Lr/(Lr+Lf) * (1 + ((Lr+Lf)/Lr*tan(x(5)))^2);

% Controls are beta_dot and acc
vec = [0 0;
       0 0;
       0 0;
       0 1;
       1 0;]; 
end



%------------- END OF CODE --------------
