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
% Ellipses -- Exclusion Zones
B1 = ((x(1) - cx1) / dx1)^2 + ((x(2) - cy1) / dy1)^2 - 1;
B2 = ((x(1) - cx2) / dx2)^2 + ((x(2) - cy2) / dy2)^2 - 1;
B3 = ((x(1) - cx3) / dx3)^2 + ((x(2) - cy3) / dy3)^2 - 1;
B4 = ((x(1) - cx4) / dx4)^2 + ((x(2) - cy4) / dy4)^2 - 1;
vals = [B1; B2; B3; B4];

% Interagent Safety


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

% This is a test
dB1dx = [2 / dx1^2 * (x(1) - cx1)    2 / dy1^2 * (x(2) - cy1)];
dB2dx = [2 / dx2^2 * (x(1) - cx2)    2 / dy2^2 * (x(2) - cy2)];
dB3dx = [2 / dx3^2 * (x(1) - cx3)    2 / dy3^2 * (x(2) - cy3)];
dB4dx = [2 / dx4^2 * (x(1) - cx4)    2 / dy4^2 * (x(2) - cy4)];

vals = [dB1dx; dB2dx; dB3dx; dB4dx];
end





%------------- END OF CODE --------------
