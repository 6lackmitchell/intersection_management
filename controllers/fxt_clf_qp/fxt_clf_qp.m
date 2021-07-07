function [u] = fxt_clf_qp(t,x,xg,cost,dynamics)
%fxt_clf_qp - Controller based on FxT-CLF-QP Formulation
%This controller solves a Quadratic Program with a Fixed-Time Control
%Lyapunov Function (CLF)-based performance constraint, i.e. the controller
%drives the state to the goal set within a fixed-time.
%
% Syntax:  [u] = clf_qp(t,x,xg,dynamics)
%
% Inputs:
%    t - current time in sec
%    x - current state vector -- ROW vector
%    xg - goal state vector -- ROW vector
%    dynamics - current state vector -- ROW vector
%
% Outputs:
%    u - control input - ROW vector
%
% Example: 
%    [u(k+1,:),d] = clf_qp(t,x)
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
run('control_params.m')
options = optimoptions('quadprog','Display','off');

[f,g] = dynamics.fg;
Nu = size(g(t,x),2);

% Load Optimization Cost Fcn
min_norm_control = zeros(Nu,1);
q = ones(Nu,1);
[Q,p] = cost(min_norm_control,q,Nu,0,0);

% Performance Condition: CLF Derivatives
LfV = dVdx(t,x-xg) * f(t,x);
LgV = dVdx(t,x-xg) * g(t,x);
FV  = -c1*V(t,x-xg)^e1 - c2*V(t,x-xg)^e2;

% Generate Constraint Matrix
A  = LgV;
b  = FV - LfV;
    
% 1/2*x^T*Q*x + p*x subject to Ax <= b
sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
if isempty(sol)
    A
    b
end
u   = sol(1:Nu);

end




%------------- END OF CODE --------------
