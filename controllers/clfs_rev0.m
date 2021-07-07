function [V_handle,dVdx_handle] = clfs()
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
V_handle    = @V;
dVdx_handle = @dVdx;

end

function val = V(t,x,xo)
%B - Control Lyapunov Function
%This function computes the current value(s) of the safety-defining CBFs 
%
% Syntax:  val = V(t,x)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%
% Outputs:
%    val - CLF value - float
%
% Example: 
%    val = V(t,x)
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

% Simple Quadratic CLF
W = eye(length(x));
val = 1/2 * x * W * x' + V_aug(t,x,xo);

end

function vals = dVdx(t,x)
%dBdx - partial derivative of CLF with respect to state (x)
%
% Syntax:  vals = dVdx(t,x)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%
% Outputs:
%    vals - dVdx values - ROW vector
%
% Example: 
%    vals = dVdx(t,x)
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

% Simple Quadratic CLF
W = eye(length(x));
vals = x * W;

end


function val = V_aug(t,x,xo)
%B - Control Lyapunov Function
%This function computes the current value(s) of the safety-defining CBFs 
%
% Syntax:  val = V(t,x)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%
% Outputs:
%    val - CLF value - float
%
% Example: 
%    val = V(t,x)
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

% Simple Quadratic CLF
comps = length(xo);
component_vals = zeros(comps,1);
for cc = 1:comps
    y = (x(1) - x(2*cc+1))*(xo(4*cc+3) - xo()) + ()*();
    component_vals(cc) = weibull(y,k,l) * smooth_ramp(z,a);
val = sum(component_vals);

end

function vals = dVdx(t,x)
%dBdx - partial derivative of CLF with respect to state (x)
%
% Syntax:  vals = dVdx(t,x)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%
% Outputs:
%    vals - dVdx values - ROW vector
%
% Example: 
%    vals = dVdx(t,x)
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

% Simple Quadratic CLF
W = eye(length(x));
vals = x * W + weibull_partial(;

end




%------------- END OF CODE --------------
