function [xdot,ydot] = path_following_fxt_clf_qp(t,x,settings)
%path_following_fxt_clf_qp - Controller based on analytical FxT Formulation
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

mode  = 'dynamic';

Gamma = settings.Gamma;
e1    = settings.e1;
e2    = settings.e2;
T     = settings.T;
r     = settings.r;
rdot  = settings.rdot;
rddot = settings.rddot;
cost  = settings.cost;

if strcmp(mode,'kinematic')
    newx  = x(1:2);
    xi    = r;
    xidot = rdot;
    A     = [0 0; 0 0];
    B     = [1 0; 0 1];
    
elseif strcmp(mode,'dynamic')
    % State for double integrator dynamics
    newx  = [x(1) x(2) x(4)*cos(x(3)) x(4)*sin(x(3))];
    
    xi    = [r rdot];
    xidot = [rdot rddot];
    A     = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
    B     = [0 0; 0 0; 1 0; 0 1];
    
end


a     = 1;
b     = (1 - e1) / ((T * a * (1 - e1))*(e2 - 1));

V     = 1/2 * (xi - newx) * (xi - newx)';
LfV   = (xi - newx) * xidot' - (xi - newx)*(A*newx');
LgV   = -(xi - newx) * B;
FV    = -a*V^e1 - b*V^e2;

Av    = LgV;
bv    = FV - LfV;

Nu       = 2;
min_norm = zeros(Nu,1);
q        = ones(Nu,1);
[Q,p]    = cost(min_norm,q,Nu,0,0);
options  = optimoptions('quadprog','Display','off');

% Solve QP
sol = quadprog(Q,p,Av,bv,[],[],[],[],[],options);
if isempty(sol)
    Av
    bv
end
u   = sol(1:2);

% Advance dynamics to obtain new velocity
run('timing.m')
xdot = newx(3) + u(1)*dt;
ydot = newx(4) + u(2)*dt;

end




%------------- END OF CODE --------------
