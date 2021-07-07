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
run('control_params.m')

% Ellipses -- Exclusion Zones
B1 = ellipse(x,cx1,dx1,cy1,dy1);
B2 = ellipse(x,cx2,dx2,cy2,dy2);
B3 = ellipse(x,cx3,dx3,cy3,dy3);
B4 = ellipse(x,cx4,dx4,cy4,dy4);

% Interagent Safety
Bn = zeros(size(xo,1),1);
for aa = 1:size(xo,1)
    Bn(aa) = interagent(t,x,xo(aa,:),R);
end

% Compiled CBF Vector
vals = [B1; B2; B3; B4; Bn];

% 2nd Order CBFs
KB = 0.5;
B1dot = partial_ellipse_partial_x(x,cx1,dx1,cy1,dy1) * x(3:4)';
B2dot = partial_ellipse_partial_x(x,cx2,dx2,cy2,dy2) * x(3:4)';
B3dot = partial_ellipse_partial_x(x,cx3,dx3,cy3,dy3) * x(3:4)';
B4dot = partial_ellipse_partial_x(x,cx4,dx4,cy4,dy4) * x(3:4)';
Bndot = zeros(size(xo,1),1);
C1    = B1dot + KB*B1;
C2    = B2dot + KB*B2;
C3    = B3dot + KB*B3;
C4    = B4dot + KB*B4;
Cn    = zeros(size(xo,1),1);
for aa = 1:size(xo,1)
    Bndot(aa) = partial_interagent_partial_x(t,x,xo(aa,:)) * (x(3:4) - xo(aa,3:4))';
    Cn(aa)    = Bndot(aa) + KB*Bn(aa);
end

% Compiled CBF Vector
vals = [C1; C2; C3; C4; Cn];

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
run('control_params.m')

% Ellipses -- Exclusion Zones
dB1dx = partial_ellipse_partial_x(x,cx1,dx1,cy1,dy1);
dB2dx = partial_ellipse_partial_x(x,cx2,dx2,cy2,dy2);
dB3dx = partial_ellipse_partial_x(x,cx3,dx3,cy3,dy3);
dB4dx = partial_ellipse_partial_x(x,cx4,dx4,cy4,dy4);

% Interagent Safety
dBndx = zeros(size(xo,1),2);
for aa = 1:size(xo,1)
    dBndx(aa,:) = partial_interagent_partial_x(t,x,xo(aa,:));
end

vals = [dB1dx; dB2dx; dB3dx; dB4dx; dBndx];

% 2nd Order CBFs
KB = 0.5;
dC1dx = [2*x(3)/dx1^2+KB*dB1dx(1) 2*x(4)/dy1^2+KB*dB1dx(2) dB1dx];
dC2dx = [2*x(3)/dx2^2+KB*dB2dx(1) 2*x(4)/dy2^2+KB*dB2dx(2) dB2dx];
dC3dx = [2*x(3)/dx3^2+KB*dB3dx(1) 2*x(4)/dy3^2+KB*dB3dx(2) dB3dx];
dC4dx = [2*x(3)/dx4^2+KB*dB4dx(1) 2*x(4)/dy4^2+KB*dB4dx(2) dB4dx];

dCndx    = zeros(size(xo,1),nStates);
for aa = 1:size(xo,1)
    dCndx(aa,:)    = [2*(x(3)-xo(aa,3))+KB*dBndx(1) 2*(x(4)-xo(aa,4))+KB*dBndx(2) dBndx(aa,:)];
end

% Compiled CBF Vector
vals = [dC1dx; dC2dx; dC3dx; dC4dx; dCndx];

end

function [B] = ellipse(x,cx,dx,cy,dy)
B = ((x(1) - cx) / dx)^2 + ((x(2) - cy) / dy)^2 - 1;
end

function [dBdx] = partial_ellipse_partial_x(x,cx,dx,cy,dy)
dBdx = [2 / dx^2 * (x(1) - cx)    2 / dy^2 * (x(2) - cy)];
end

function [D] = interagent(t,x,xo,R)
D = (x(1) - xo(1))^2 + (x(2) - xo(2))^2 - R^2;
end

function [D] = partial_interagent_partial_x(t,x,xo)
D = [2*(x(1) - xo(1))  2*(x(2) - xo(2))];
end




%------------- END OF CODE --------------
