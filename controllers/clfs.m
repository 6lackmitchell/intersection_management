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

function val = V(t,x,xd,aa)
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
val = 1/2 * x * W * x';
% val = val + V_aug(t,x,xd,aa);

end

function vals = dVdx(t,x,xd,aa)
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
% vals = vals + dVaugdx(t,x,xd,aa);

end


function val = V_aug(t,x,xd,aa)
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
k = 3.0;
l = 0.5;
a = 1000;

comps = size(xd,1);
component_vals = zeros(comps,1);
for cc = 1:comps
    if aa == cc
        continue
    end
    [y,z] = get_y_z(xd,aa,cc);
    
    component_vals(cc) = weibull(y,k,l) * smooth_ramp(z,a);
end

val = sum(component_vals);

end

function vals = dVaugdx(t,x,xd,aa)
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

W = eye(length(x));

k = 3.0;
l = 0.5;
a = 1000;

comps = size(xd,1);
component_vals = zeros(comps,2);
for cc = 1:comps
    if aa == cc
        continue
    end
    
    Nx = 0; Ny = 0;
    V_mag = sqrt((xd(cc,3)-xd(aa,3))^2 + (xd(cc,4)-xd(aa,4))^2);
    if V_mag > 0
        Nx    = (xd(cc,3) - xd(aa,3)) / V_mag;
        Ny    = (xd(cc,4) - xd(aa,4)) / V_mag;
    end
    F     = 1 / sqrt((xd(aa,1)-xd(cc,1))^2 + (xd(aa,2)-xd(cc,2))^2);
    dFdx  = -F^3*(xd(aa,1)-xd(cc,1));
    dFdy  = -F^3*(xd(aa,2)-xd(cc,2));
    
    [y,z] = get_y_z(xd,aa,cc);
    dydx  = [xd(cc,3) - xd(aa,3); xd(cc,4) - xd(aa,4)]';
    dzdx  = [Nx*F + Nx*(xd(aa,1)-xd(cc,1))*dFdx + Ny*(xd(aa,2)-xd(cc,2))*dFdx;
             Ny*F + Ny*(xd(aa,2)-xd(cc,2))*dFdy + Nx*(xd(aa,1)-xd(cc,1))*dFdy]';
    
    term1 = weibull_partial(y,k,l) * dydx * smooth_ramp(z,a);
    term2 = weibull(y,k,l) * smooth_ramp_partial(z,a) * dzdx;
    component_vals(cc,:) = term1 + term2;
    if any(isnan(term1)) || any(isnan(term2))
        term1
        term2
    end
end

vals = sum(component_vals);

end


function [y,z] = get_y_z(xd,aa,cc)
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

dx  = xd(aa,1) - xd(cc,1);
dy  = xd(aa,2) - xd(cc,2);
dvx = xd(cc,3) - xd(aa,3);
dvy = xd(cc,4) - xd(aa,4);
y = dx*dvx + dy*dvy;
if (dx == 0 && dy ==0) || (dvx == 0 && dvy == 0)
    z = 0;
else
    z = y / (sqrt(dx^2 + dy^2) * sqrt(dvx^2 + dvy^2));
end

end



%------------- END OF CODE --------------
