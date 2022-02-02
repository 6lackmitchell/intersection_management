function [xdot] = double_integrator(t,x,u,settings)
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
nAgents      = size(x,1);
drift_term   = zeros(nAgents,size(x,2));
control_term = zeros(nAgents,size(x,2));

for aa = 1:nAgents
    drift_term(aa,:)   = f(t,x(aa,:));
    control_term(aa,:) = g(t,x(aa,:)) * u(aa,:)';
end

xdot = drift_term + control_term;


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
vec = [0 0 1 0;
       0 0 0 1;
       0 0 0 0; 
       0 0 0 0] * x';

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
% Controls are ax, ay
vec = [0 0;
       0 0;
       1 0;
       0 1]; 
end



%------------- END OF CODE --------------
