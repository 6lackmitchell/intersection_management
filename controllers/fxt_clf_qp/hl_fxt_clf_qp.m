function [u] = hl_fxt_clf_qp(t,x,settings)
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

Na    = size(x,1);
xGoal = settings.xg;
[f,g] = settings.dyn;
cost  = settings.cost;
uLast = settings.uLast;
Nu    = size(uLast,2);
Nua   = Nu*Na;

% Load Optimization Cost Fcn
min_norm_control = zeros(Nua,1);
min_deviation_control = reshape(uLast,[Nua 1]);
const_speed_control = 10*(xGoal(1,1:2) - x(1,1:2))/norm(xGoal(1,1:2) - x(1,1:2));
model_control = const_speed_control';
q = ones(Nua,1);
[Q,p] = cost(model_control,q,Nua,0,0);

A = kron(zeros(Na),zeros(1,Nu));
b = zeros(Na,1);
for aa = 1:Na
    xx = x(aa,1:2);
    xg = xGoal(aa,:);
    xd = x(aa,:);
    
    % Performance Condition: CLF Derivatives
    LfV = dVdx(t,xx-xg,xd,aa) * f(t,xx);
    LgV = dVdx(t,xx-xg,xd,aa) * g(t,xx);
    FV  = -c1*V(t,xx-xg,xd,aa)^e1 - c2*V(t,xx-xg,xd,aa)^e2;

    % Generate Constraint Matrix
%     A(aa,(Nu*aa - Nu)+(1:2)) = LgV;
    A(aa,Nu*aa+(-(Nu-1):-(Nu-2))) = LgV;
    b(aa)              = FV - LfV;
end

if sum(any(isnan(A))) || any(isnan(b))
    A
    b
end
    
% 1/2*x^T*Q*x + p*x subject to Ax <= b
sol = quadprog(Q,p,A,b,[],[],[],[],[],options);
if isempty(sol)
    A
    b
end
u   = sol(1:Nua);


end




%------------- END OF CODE --------------
