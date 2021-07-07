function [B_handle,dBdx_handle] = cbfs()
%B - Control Barrier Function
%This function computes the current value(s) of the safety-defining CBFs 
%
% Syntax:  vals = B(t,x)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%
% Outputs:
%    vals - CBF values - COLUMN vector
%
% Example: 
%    vals = B(t,x)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% June 2021; Last revision: 22-Jun-2021
%------------- BEGIN CODE --------------
% This is a test
B_handle    = @B;
dBdx_handle = @dBdx;

end

function vals = B(t,x,xo)
%B - Control Barrier Function
%This function computes the current value(s) of the safety-defining CBFs 
%
% Syntax:  vals = B(t,x)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%
% Outputs:
%    vals - CBF values - COLUMN vector
%
% Example: 
%    vals = B(t,x)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% June 2021; Last revision: 22-Jun-2021
%------------- BEGIN CODE --------------
run('control.m')

vals = [];
for ii = 1:size(x,2)
    agent_vals = [];
    B1 = ((x(ii,1) - cx1) / dx1)^2 + ((x(ii,2) - cy1) / dy1)^2 - 1;
    B2 = ((x(ii,1) - cx2) / dx2)^2 + ((x(ii,2) - cy2) / dy2)^2 - 1;
    agent_vals = [B1 B2];
    for jj = 1:size(x,2)
        if (ii==jj)
            continue
        end
        
        Bn = norm(x(ii)-x(jj)) - R^2;
        agent_vals = [agent_vals Bn];
    end
    
    vals = [vals; agent_vals];
end

% % Ellipses -- Exclusion Zones
% B1 = ((x(1) - cx1) / dx1)^2 + ((x(2) - cy1) / dy1)^2 - 1;
% B2 = ((x(1) - cx2) / dx2)^2 + ((x(2) - cy2) / dy2)^2 - 1;
% vals = [B1; B2];
% 
% % Interagent Safety


end

function vals = dBdx(t,x,xo)
%dBdx - partial derivative of CBFs with respect to state (x)
%
% Syntax:  vals = dBdx(t,x)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%
% Outputs:
%    vals - dBdx values - COLUMN vector
%
% Example: 
%    vals = dBdx(t,x)
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: OTHER_FUNCTION_NAME1,  OTHER_FUNCTION_NAME2
% Author: Mitchell Black
% Email: mblackjr@umich.edu
% Website: http://www.blackmitchell.com
% June 2021; Last revision: 22-Jun-2021
%------------- BEGIN CODE --------------
run('control.m')

vals = [];
for ii = 1:size(x,2)
    agent_vals = [];
    dB1dx = [2 / dx1^2 * (x(1) - cx1)    2 / dy1^2 * (x(2) - cy1)];
    dB2dx = [2 / dx2^2 * (x(1) - cx2)    2 / dy2^2 * (x(2) - cy2)];

    agent_vals = [dB1dx; dB2dx];
    for jj = 1:size(x,2)
        if (ii==jj)
            continue
        end
        
        dBndx = [(x(ii,1)-x(jj,1))/norm(x(ii,1:2)-x(jj,1:2))   (x(ii,2)-x(jj,2))/norm(x(ii,1:2)-x(jj,1:2))];
        agent_vals = [agent_vals; Bn];
    end
    
    vals = cat(3,vals,agent_vals);
end

% % This is a test
% dB1dx = [2 / dx1^2 * (x(1) - cx1)    2 / dy1^2 * (x(2) - cy1)];
% dB2dx = [2 / dx2^2 * (x(1) - cx2)    2 / dy2^2 * (x(2) - cy2)];
% 
% vals = [dB1dx; dB2dx];
end





%------------- END OF CODE --------------
