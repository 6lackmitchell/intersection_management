function [xdot,f,g] = dynamics(mode,t,x,u)
%dynamics - State-derivative of the chosen dynamics
%This function takes in the relevant information required to compute the
%state time-derivative necessary for stepping the system state forward in
%time.
%
% Syntax:  [xdot] = dynamics(mode,t,x,u)
%
% Inputs:
%    mode - name of specific dynamics to simulate -- str
%    t - current time in sec -- float
%    x - current state vector -- ROW vector
%    u - control input -- ROW vector
%
% Outputs:
%    xdot - vector of state-derivatives - ROW vector
%
% Example: 
%    x(k+1) = x(k) + dt*dynamics('double_integrator',t,x,u)
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
dynamic_mode = str2func(mode);
[f,g] = dynamic_mode();

nAgents = size(x,1);

% All agents states/control passed
% if nAgents > 1
drift_term   = zeros(nAgents,size(x,2));
control_term = zeros(nAgents,size(x,2));
for aa = 1:nAgents
    drift_term(aa,:) = f(t,x(aa,:));
    if(all(u==0))
        control_term(aa,:) = 0 * f(t,x(aa,:));
    else
        control_term(aa,:) = g(t,x(aa,:)) * u(aa,:)';
    end
end
    
% % Individual agent passed
% else
%     drift_term = f(t,x);
%     if(all(u==0))
%         control_term = 0 * f(t,x);
%     else
%         control_term = g(t,x) * u;
%     end
%     
% end

xdot = drift_term + control_term;

end

%------------- END OF CODE --------------
